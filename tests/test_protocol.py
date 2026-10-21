from neato import protocol as p


def test_strip_echo_and_csv():
    raw = "GetCharger\r\nLabel,Value\r\nFuelPercent,87\r\nExtPwrPresent,1,\r\n"
    body = p.strip_echo(raw, "GetCharger")
    assert not body.startswith("GetCharger")
    assert p.csv_values(body)["ExtPwrPresent"] == "1"
    charger = p.parse_charger(body)
    assert charger.battery_percent == 87 and charger.external_power


def test_state_and_error():
    state = p.parse_state("Current UI State is: UIMGR_STATE_HOUSECLEANINGRUNNING\nCurrent Robot State is: ST_C_HouseCleaning\n")
    assert state.ui_state == "UIMGR_STATE_HOUSECLEANINGRUNNING"
    assert p.is_cleaning(state.ui_state)
    assert p.parse_error("") is None
    assert p.parse_error("200 -  (UI_ALERT_INVALID)") is None
    err = p.parse_error("200 -  (UI_ALERT_INVALID)\n243 - (UI_ERROR_DUST_BIN_MISSING)")
    assert err.code == 243 and "DUST_BIN" in err.message
    assert p.ha_vacuum_state("UIMGR_STATE_IDLE", None, err) == "error"


def test_pose_formats():
    assert p.parse_pose("Pose: X=1.234 Y=-0.5 Theta=93.1 Time=12.0") == p.Pose(1.234, -0.5, 93.1, 12.0)
    assert p.parse_pose("X = 1, Y = 2, Theta = -3.5, Time = 4") == p.Pose(1, 2, -3.5, 4)
    assert p.parse_pose("Unknown Cmd") is None


def test_lds_scan():
    raw = "AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX\n0,221,1400,0\n1,0,0,8035\n359,220,1421,0\nROTATION_SPEED,5.00\n"
    points, speed = p.parse_lds_scan(raw)
    assert speed == 5.0
    assert [pt.angle_deg for pt in points] == [0, 1, 359]
    assert points[1].error == 0x8035


def test_skey_matches_reference_implementation():
    # Values produced by compiling OpenNeato's C++ computeSKey
    assert p.compute_skey("ABC12345DE,0123456789AB,D") == "b20e9dfade296f1210da702bf"
    assert p.compute_skey("OPS01234AA,f0fe6b123456,D") == "e40fc9acdc7e68171bd7045fa"
    assert p.compute_skey("no-comma") is None
