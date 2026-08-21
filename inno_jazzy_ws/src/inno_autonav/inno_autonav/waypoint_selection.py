"""Pure helpers for named waypoint selection without ROS dependencies."""

import re


_WAYPOINT_PATTERN = re.compile(r'w[1-9][0-9]*', re.IGNORECASE)


def waypoint_names_from_document(document):
    """Return stable labels for named YAML or positional pose snapshots."""
    if 'poses' in document:
        raw_poses = document['poses']
    else:
        raw_poses = document.get('semantic_points', {})
    if isinstance(raw_poses, dict):
        names = [str(name).strip() for name in raw_poses]
    elif isinstance(raw_poses, list):
        names = [f'w{index}' for index in range(1, len(raw_poses) + 1)]
    else:
        raise ValueError('poses 또는 semantic_points 형식이 올바르지 않습니다.')
    if any(not name for name in names):
        raise ValueError('waypoint 이름은 비어 있을 수 없습니다.')
    normalized = [name.casefold() for name in names]
    if len(set(normalized)) != len(normalized):
        raise ValueError('waypoint 이름은 대소문자를 무시하고 고유해야 합니다.')
    return names


def resolve_named_waypoints(text, available_names):
    """Resolve a named selection to canonical names and queue indices."""
    requested = [item.strip() for item in text.split(',') if item.strip()]
    if len(requested) < 2:
        raise ValueError('MODE 2는 waypoint를 2개 이상 입력해야 합니다.')
    for name in requested:
        if _WAYPOINT_PATTERN.fullmatch(name) is None:
            raise ValueError(f'MODE 2 waypoint 이름 형식 오류: {name!r}')
    lookup = {
        name.casefold(): (index, name)
        for index, name in enumerate(available_names)
    }
    missing = [name for name in requested if name.casefold() not in lookup]
    if missing:
        raise ValueError('존재하지 않는 waypoint: ' + ','.join(missing))
    indices = [lookup[name.casefold()][0] for name in requested]
    names = [lookup[name.casefold()][1] for name in requested]
    return names, indices
