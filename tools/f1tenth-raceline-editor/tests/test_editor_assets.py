from pathlib import Path


def test_sector_editor_has_safety_guards() -> None:
    root = Path(__file__).parents[1] / "src/f1tenth_raceline/web"
    html = (root / "index.html").read_text()
    js = (root / "editor.js").read_text()
    assert '<script src="/editor.js"></script>' in html
    assert "bestD <= 35*35 ? best : null" in js
    assert "!data.shortest_available" in js
    assert "overtaking editor is disabled" in js
