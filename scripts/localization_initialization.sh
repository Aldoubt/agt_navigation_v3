#!/usr/bin/env bash
# Sourced by run_field_stack.sh after its lifecycle helpers. V1 has no manual
# initial-pose or RTK seed path: failed automatic localization exits before Nav2.

initialize_localization() {
  if [[ "$LOCALIZATION_MODE" != auto ]]; then
    printf 'Invalid localization mode for V1: %s (automatic global relocalization only)\n' \
      "$LOCALIZATION_MODE" >&2
    return 2
  fi
  relocalize_until_ready relocalize
}
