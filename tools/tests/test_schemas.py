from vg_tools import schemas


def test_c_string_literal_escapes_every_byte():
    assert schemas.c_string_literal(b'A"\x00') == '    "\\x41\\x22\\x00"'


def test_c_string_literal_splits_long_data():
    lines = schemas.c_string_literal(bytes(range(40)), width=40).splitlines()
    assert len(lines) == 4
    assert all(len(line) <= 46 for line in lines)


def test_render_builds_lookup_table():
    source = schemas.render({"vg.Imu": b"\x01\x02"})
    assert 'constexpr char kSchema0[] =\n    "\\x01\\x02";' in source
    assert '{"vg.Imu", {kSchema0, sizeof(kSchema0) - 1}},' in source


def test_every_schema_file_exists():
    root = schemas.Path(__file__).resolve().parents[2]
    for proto in schemas.SCHEMAS.values():
        assert (root / "schemas" / proto).is_file(), proto
