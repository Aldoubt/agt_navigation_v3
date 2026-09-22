import csv
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from agt_navigation_runtime.record_writer import RecordWriter


def mission(tmp_path, name='m1'):
    source=tmp_path/'mission.yaml';source.write_text('version: 1\n')
    view=SimpleNamespace(tag='front',required=True,save_image=True)
    return SimpleNamespace(mission_id=name,map_id='test_map',source_file=str(source),
                           points=[SimpleNamespace(id='P001',views=[view]),
                                   SimpleNamespace(id='RETURN_HOME',views=[])])


@pytest.mark.parametrize('state',['completed','failed','canceled','interrupted'])
def test_all_terminal_states_persist_atomically(tmp_path, state):
    w=RecordWriter(str(tmp_path/'records'),mission(tmp_path))
    w.append({'point_id':'P001','view_tag':'front','camera_error_code':301,
              'camera_error_message':'timeout','capture_status':'failed'})
    assert w.finalize(state,completed_points=0,error_code=301,message='detail',
                      child_actions_terminal=True)
    m=json.loads(w.manifest_path.read_text())
    assert m['status']==state and m['capture_records']==1 and m['planned_points']==2
    assert len(m['planned_views'])==1 and m['error_code']==301
    assert m['finished_at_utc'] and not w.manifest_path.with_suffix('.json.tmp').exists()
    events=[json.loads(l) for l in w.events_path.read_text().splitlines()]
    assert events[-1]['event']=='mission_terminal'
    assert not w.finalize('completed',completed_points=2,error_code=0,message='late overwrite')
    assert json.loads(w.manifest_path.read_text())['status']==state


def test_failed_view_detail_is_retained_without_changing_csv_contract(tmp_path):
    w=RecordWriter(str(tmp_path/'records'),mission(tmp_path))
    w.append({'point_id':'P001','view_tag':'front','camera_error_code':301,
              'camera_error_message':'no fresh image','capture_status':'failed'})
    assert list(csv.DictReader(w.csv_path.open()))[0]['camera_error_code']=='301'
    row=json.loads(w.jsonl_path.read_text())
    assert row['camera_error_message']=='no fresh image' and row['capture_status']=='failed'


def test_missing_or_empty_image_is_not_silently_adopted(tmp_path):
    w=RecordWriter(str(tmp_path/'records'),mission(tmp_path))
    with pytest.raises(FileNotFoundError):w.adopt_image(str(tmp_path/'missing.png'),'P001','front')
    p=tmp_path/'empty.png';p.write_bytes(b'')
    with pytest.raises(FileNotFoundError):w.adopt_image(str(p),'P001','front')


def test_image_archive_keeps_existing_paths(tmp_path):
    w=RecordWriter(str(tmp_path/'records'),mission(tmp_path))
    p=tmp_path/'photo.png';p.write_bytes(b'fixture bytes; not a real camera image')
    a=Path(w.adopt_image(str(p),'P001','front'))
    b=Path(w.adopt_image(str(p),'P001','front'))
    assert a!=b and a.read_bytes()==b.read_bytes()==p.read_bytes()
    assert a.parent==w.images_dir/'P001'


@pytest.mark.parametrize('unsafe',['../escape','/absolute','..','a/b','a\\b'])
def test_mission_id_cannot_escape_record_root(tmp_path,unsafe):
    with pytest.raises(ValueError):RecordWriter(str(tmp_path/'records'),mission(tmp_path,unsafe))


def test_terminal_manifest_write_failure_does_not_commit_in_memory(tmp_path, monkeypatch):
    w=RecordWriter(str(tmp_path/'records'),mission(tmp_path))
    real_write=w._write_manifest
    def fail(_value=None):
        raise OSError('simulated disk failure')
    monkeypatch.setattr(w,'_write_manifest',fail)
    with pytest.raises(OSError):w.finalize('completed',completed_points=2,error_code=0,message='done')
    assert w.manifest['status']=='running'
    assert json.loads(w.manifest_path.read_text())['status']=='running'
    monkeypatch.setattr(w,'_write_manifest',real_write)
    w.finalize('failed',completed_points=0,error_code=1401,message='write failed')
    assert json.loads(w.manifest_path.read_text())['status']=='failed'


def test_event_append_failure_cannot_reverse_an_atomic_terminal_commit(tmp_path, monkeypatch):
    w=RecordWriter(str(tmp_path/'records'),mission(tmp_path))
    def fail(*_args,**_kwargs):
        raise OSError('event disk failure')
    monkeypatch.setattr(w,'event',fail)
    assert w.finalize('failed',completed_points=0,error_code=301,message='camera failed')
    assert json.loads(w.manifest_path.read_text())['status']=='failed'
    assert w.last_event_error=='event disk failure'
