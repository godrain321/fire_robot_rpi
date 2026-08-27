from inno_autonav.mode5_route_preview import compute_mode5_preview
from inno_autonav.project_paths import project_path


def test_exit1_blocked_static_only_preview_matches_deployed_route():
    preview, _ = compute_mode5_preview(
        project_path('maps', 'inno_map_nav.yaml'),
        project_path('docs', 'full_map_waypoints_1m_numbered.yaml'),
        project_path(
            'inno_jazzy_ws', 'src', 'inno_autonav', 'config',
            'semantic_points.yaml'
        ),
        ('EXIT1',),
    )

    assert preview.blocked_exit_ids == ('EXIT1',)
    assert preview.selected_exit_id == 'EXIT2'
    assert preview.exit_evaluation_waypoints == (
        'w43', 'w44', 'w55', 'w61', 'w65', 'w75', 'w77', 'w110'
    )
    assert preview.reference_waypoints == (
        'w42', 'w44', 'w56', 'w60', 'w66', 'w74', 'w78', 'w109'
    )
    assert preview.drive_waypoints == ('w42', 'w109')
    assert preview.drive_points[-1] == (14.707, -6.82)
