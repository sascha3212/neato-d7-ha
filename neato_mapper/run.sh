#!/usr/bin/with-contenv bashio
# Hands the MQTT broker details from the Supervisor to the mapper.

export MQTT_HOST="$(bashio::services mqtt 'host')"
export MQTT_PORT="$(bashio::services mqtt 'port')"
export MQTT_USERNAME="$(bashio::services mqtt 'username')"
export MQTT_PASSWORD="$(bashio::services mqtt 'password')"

ARGS=()
if bashio::config.true 'debug'; then
  ARGS+=(--debug)
fi

exec python3 -m neato "${ARGS[@]}"
