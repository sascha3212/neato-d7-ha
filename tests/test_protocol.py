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


# Answers captured from a real Botvac D7 (software 4.4.0-72) on 2026-10-21,
# including the echoed command line the robot prints first
REAL_POSE = "GetRobotPos Smooth\r\nRobot Smooth pose: X=0.000, Y=0.000, Theta=0.000, Time=38327.516362 \r\n"
REAL_STATE = "GetState\r\nCurrent UI State is: UIMGR_STATE_STANDBY\r\nCurrent Robot State is: ST_C_Standby\r\n"
REAL_ERR = "GetErr\r\nError\r\n200 -  (UI_ALERT_INVALID)\r\nAlert\r\n200 -  (UI_ALERT_INVALID)\r\nUSB state \r\n NOT connected \r\n"
REAL_CHARGER = (
    "GetCharger\r\nLabel,Value\r\nFuelPercent,98\r\nBatteryOverTemp,0\r\nChargingActive,0\r\n"
    "ChargingEnabled,1\r\nConfidentOnFuel,0\r\nExtPwrPresent,1\r\nVBattV,16.57\r\n"
)


def test_real_d7_answers():
    pose = p.parse_pose(p.strip_echo(REAL_POSE, "GetRobotPos Smooth"))
    assert pose == p.Pose(0.0, 0.0, 0.0, 38327.516362)
    state = p.parse_state(p.strip_echo(REAL_STATE, "GetState"))
    assert state.ui_state == "UIMGR_STATE_STANDBY"
    assert p.parse_error(p.strip_echo(REAL_ERR, "GetErr")) is None
    charger = p.parse_charger(p.strip_echo(REAL_CHARGER, "GetCharger"))
    assert charger.battery_percent == 98 and charger.external_power and not charger.charging
    assert p.ha_vacuum_state(state.ui_state, charger, None) == "docked"


D7_USER_SETTINGS = (
    "GetUserSettings\r\nLanguage, EL_NONE\r\nClickSounds, OFF\r\nLED, ON\r\nWall Enable, ON\r\n"
    "Eco Mode, ON\r\nIntenseClean, OFF\r\nWiFi, ON\r\nMelody Sounds, OFF\r\nWarning Sounds, ON\r\n"
    "Bin Full Detect, ON\r\nFilter Change Time (seconds), 43200\r\nSchedule is Disabled\r\n"
)


def test_user_settings():
    s = p.parse_user_settings(p.strip_echo(D7_USER_SETTINGS, "GetUserSettings"))
    assert p.fan_speed(s) == "eco"
    assert p.toggle_state(s, "Wall Enable") is True
    assert p.toggle_state(s, "ClickSounds") is False
    assert p.toggle_state(s, "LED") is True  # lights visible
    assert p.toggle_state(s, "Missing") is None
    assert p.set_toggle("StealthLED", True, True) == "SetUserSettings StealthLED OFF"
    assert p.set_toggle("WallEnable", False, False) == "SetUserSettings WallEnable OFF"


def test_version_fields():
    d7 = p.csv_values("Component,Major,Minor,Build,Aux\r\nModel,BotVacD7Connected,905-0415,\r\nSoftware,4,4,0,72,0\r\n")
    assert p.product_model(d7) == "BotVacD7Connected"
    assert p.software_version(d7) == "4.4.0-72"
    old = p.csv_values("Product Model,BotVacD5Connected,,\r\nSoftware,4,5,3\r\n")
    assert p.product_model(old) == "BotVacD5Connected"
    assert p.software_version(old) == "4.5.3"
