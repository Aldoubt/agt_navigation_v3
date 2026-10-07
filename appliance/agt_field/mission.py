"""Action-result-driven mission state machine. Never publishes velocity or TF."""

from __future__ import annotations

import time

from .contracts import ContractError, route


class Mission:
    def __init__(self, send, cancel, now=time.monotonic):
        self.send = send
        self.cancel_goal = cancel
        self.now = now
        self.state = "IDLE"
        self.detail = ""
        self.index = 0
        self.epoch = 0
        self.goal_active = False
        self.cancel_pending = False
        self.localization_ready = False
        self.gateway_ready = False
        self.binding = None
        self.data = None
        self.remaining = 0.0
        self.deadline = 0.0
        self.resume_state = None
        self.events = []

    def transition(self, state, detail=""):
        self.state, self.detail = state, detail
        self.events.append(dict(state=state, index=self.index, detail=detail, time=self.now()))

    def bind(self, binding):
        if self.state in {"RUNNING", "NAVIGATING", "DWELLING", "PAUSED"} or self.cancel_pending:
            raise ContractError("cannot change map during mission/cancellation")
        self.binding = dict(binding)
        self.data = None
        self.transition("IDLE")

    def load(self, data):
        if self.state in {"RUNNING", "NAVIGATING", "DWELLING", "PAUSED"} or self.cancel_pending:
            raise ContractError("cannot replace active route")
        if not self.binding:
            raise ContractError("no active map")
        self.data = route(data, self.binding)
        self.index = 0
        self.transition("READY")

    def readiness(self, localization, gateway):
        self.localization_ready, self.gateway_ready = bool(localization), bool(gateway)
        if self.state in {"RUNNING", "NAVIGATING", "DWELLING", "PAUSED"} and not (
            localization and gateway
        ):
            self.stop("ERROR", "localization lost" if not localization else "gateway disconnected")

    def safe(self):
        if not self.localization_ready or not self.gateway_ready or self.cancel_pending:
            raise ContractError("mission interlock: localization/gateway/cancel barrier")
        if not self.data or self.data["map"] != self.binding:
            raise ContractError("route map identity mismatch")

    def start(self):
        if self.state not in {"READY", "COMPLETED", "CANCELLED"}:
            raise ContractError("mission is not ready")
        self.safe()
        self.index = 0
        self.transition("RUNNING")
        self.navigate()

    def navigate(self):
        self.safe()
        self.epoch += 1
        self.goal_active = True
        self.transition("NAVIGATING")
        try:
            self.send(self.data["waypoints"][self.index], self.epoch)
        except Exception as exc:
            self.stop("ERROR", f"goal dispatch failed: {exc}")

    def result(self, epoch, success, detail=""):
        # Late results from paused/cancelled/replaced goals cannot advance a route.
        if epoch != self.epoch or self.state != "NAVIGATING" or not self.goal_active:
            return
        self.goal_active = False
        if not success:
            self.transition("ERROR", detail or "Nav2 failure")
            return
        self.remaining = self.data["waypoints"][self.index]["dwell_seconds"]
        self.deadline = self.now() + self.remaining
        self.transition("DWELLING")

    def tick(self):
        if self.state == "DWELLING":
            self.remaining = max(0.0, self.deadline - self.now())
            if self.remaining <= 0:
                self.index += 1
                if self.index == len(self.data["waypoints"]):
                    self.transition("COMPLETED")
                else:
                    self.navigate()

    def request_cancel(self):
        if self.goal_active:
            self.goal_active = False
            self.cancel_pending = True
            self.cancel_goal(self.epoch)

    def cancelled(self, epoch):
        if epoch == self.epoch:
            self.cancel_pending = False

    def pause(self):
        if self.state not in {"NAVIGATING", "DWELLING"}:
            raise ContractError("cannot pause this state")
        self.resume_state = self.state
        if self.state == "DWELLING":
            self.remaining = max(0.0, self.deadline - self.now())
        self.transition("PAUSED")
        self.request_cancel()

    def resume(self):
        if self.state != "PAUSED":
            raise ContractError("mission is not paused")
        self.safe()
        if self.resume_state == "DWELLING":
            self.deadline = self.now() + self.remaining
            self.transition("DWELLING")
        else:
            self.navigate()

    def stop(self, state="CANCELLED", detail="operator cancel"):
        self.transition(state, detail)
        self.request_cancel()

    def status(self):
        return dict(
            state=self.state,
            detail=self.detail,
            waypoint_index=self.index,
            dwell_remaining=self.remaining,
            cancel_pending=self.cancel_pending,
            route_id=self.data["route_id"] if self.data else "",
        )
