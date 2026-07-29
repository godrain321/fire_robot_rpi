import os
from unittest import mock

import pytest
import yaml

from inno_semantic_nav.semantic_store import (
    DuplicateNameError,
    InvalidNameError,
    InvalidSemanticFileError,
    SemanticStore,
)


def test_missing_file_creates_empty_yaml(tmp_path):
    path = tmp_path / 'semantic_points.yaml'
    document = SemanticStore(path).load()

    assert path.is_file()
    assert document == {
        'version': 1,
        'site_id': 'test_map',
        'frame_id': 'map',
        'poses': {},
        'landmarks': {},
    }
    assert yaml.safe_load(path.read_text(encoding='utf-8')) == document


def test_save_e1_pose(tmp_path):
    store = SemanticStore(tmp_path / 'semantic_points.yaml')
    store.add_pose('E1', 1.25, -0.5, 0.75, 'exit', 'main_exit')

    pose = store.load()['poses']['E1']
    assert pose['category'] == 'exit'
    assert pose['description'] == 'main_exit'
    assert pose['x'] == pytest.approx(1.25)
    assert pose['y'] == pytest.approx(-0.5)
    assert pose['yaw'] == pytest.approx(0.75)


def test_add_e2_preserves_e1(tmp_path):
    store = SemanticStore(tmp_path / 'semantic_points.yaml')
    store.add_pose('E1', 1.0, 2.0, 0.1)
    original_e1 = dict(store.load()['poses']['E1'])
    store.add_pose('E2', 3.0, 4.0, -0.2)

    poses = store.load()['poses']
    assert poses['E1'] == original_e1
    assert poses['E2']['x'] == pytest.approx(3.0)


def test_duplicate_pose_is_rejected(tmp_path):
    store = SemanticStore(tmp_path / 'semantic_points.yaml')
    store.add_pose('E1', 1.0, 2.0, 0.0)

    with pytest.raises(DuplicateNameError):
        store.add_pose('E1', 9.0, 9.0, 1.0)
    assert store.load()['poses']['E1']['x'] == pytest.approx(1.0)


def test_overwrite_pose(tmp_path):
    store = SemanticStore(tmp_path / 'semantic_points.yaml')
    store.add_pose('E1', 1.0, 2.0, 0.0)
    store.add_pose('E1', 9.0, 8.0, 7.0, overwrite=True)

    pose = store.load()['poses']['E1']
    assert pose['x'] == pytest.approx(9.0)
    assert pose['y'] == pytest.approx(8.0)
    assert -3.141592653589793 <= pose['yaw'] <= 3.141592653589793


def test_save_landmark(tmp_path):
    store = SemanticStore(tmp_path / 'semantic_points.yaml')
    store.add_landmark('MACHINE_01', 2.5, -1.5, 'machine', 'fixed_machine')

    landmark = store.load()['landmarks']['MACHINE_01']
    assert landmark == {
        'category': 'machine',
        'description': 'fixed_machine',
        'x': 2.5,
        'y': -1.5,
    }


def test_malformed_yaml_is_rejected(tmp_path):
    path = tmp_path / 'semantic_points.yaml'
    path.write_text('version: 1\nposes: [\n', encoding='utf-8')

    with pytest.raises(InvalidSemanticFileError):
        SemanticStore(path).load()


def test_unknown_top_level_key_is_preserved(tmp_path):
    path = tmp_path / 'semantic_points.yaml'
    path.write_text(
        'version: 1\nsite_id: test_map\nframe_id: map\n'
        'custom_metadata:\n  owner: safety_team\nposes: {}\nlandmarks: {}\n',
        encoding='utf-8',
    )
    store = SemanticStore(path)
    store.add_pose('E1', 0.0, 0.0, 0.0)

    assert store.load()['custom_metadata'] == {'owner': 'safety_team'}


def test_atomic_replace_produces_valid_yaml(tmp_path):
    path = tmp_path / 'semantic_points.yaml'
    store = SemanticStore(path)
    with mock.patch(
        'inno_semantic_nav.semantic_store.os.replace', wraps=os.replace
    ) as replace:
        store.add_pose('E1', 1.0, 2.0, 0.3)

    replace.assert_called()
    parsed = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert parsed['poses']['E1']['yaw'] == pytest.approx(0.3)
    assert list(tmp_path.glob('.semantic_points.yaml.*.tmp')) == []


@pytest.mark.parametrize('name', ['', 'E 1', 'E/1', '출구', 'E.1'])
def test_invalid_name_is_rejected(tmp_path, name):
    store = SemanticStore(tmp_path / 'semantic_points.yaml')
    with pytest.raises(InvalidNameError):
        store.add_pose(name, 0.0, 0.0, 0.0)
