from types import SimpleNamespace

from inno_semantic_nav.go_named_pose import _print_result_details


def test_jazzy_result_fields_are_printed(capsys):
    wrapped_result = SimpleNamespace(
        result=SimpleNamespace(error_code=42, error_msg='planner failed')
    )

    _print_result_details(wrapped_result)

    assert capsys.readouterr().out.strip() == (
        'Nav2 결과: error_code=42, error_msg=planner failed'
    )


def test_humble_style_empty_result_is_still_supported(capsys):
    _print_result_details(SimpleNamespace(result=SimpleNamespace()))

    assert capsys.readouterr().out == ''
