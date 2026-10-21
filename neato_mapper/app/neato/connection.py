"""Talks to the robot through the ESP32 serial bridge (raw serial over TCP)."""

from __future__ import annotations

import asyncio
import logging

from .protocol import TERMINATOR, echoes, strip_echo

_LOGGER = logging.getLogger(__name__)

# Pause between commands; the robot's serial task can hang when it is
# hammered back to back while it cleans
INTER_COMMAND_DELAY_S = 0.1


class RobotLink:
    """One command at a time, each answer read up to the 0x1A terminator."""

    def __init__(self, host: str, port: int, timeout: float = 5.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    async def connect(self) -> None:
        self._reader, self._writer = await asyncio.wait_for(
            asyncio.open_connection(self.host, self.port), self.timeout
        )
        _LOGGER.info("Connected to serial bridge at %s:%s", self.host, self.port)
        await self._drain_input()

    async def close(self) -> None:
        if self._writer is not None:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except OSError:
                pass
        self._reader = self._writer = None

    async def _drain_input(self) -> None:
        """Throw away boot logs or half answers left in the pipe."""
        assert self._reader is not None
        while True:
            try:
                data = await asyncio.wait_for(self._reader.read(4096), 0.3)
            except asyncio.TimeoutError:
                return
            if not data:
                return

    async def command(self, cmd: str, timeout: float | None = None) -> str:
        """Send one command and return its answer without the echo line."""
        async with self._lock:
            if not self.connected:
                await self.connect()
            assert self._reader is not None and self._writer is not None
            try:
                self._writer.write(cmd.encode("ascii") + b"\n")
                await self._writer.drain()
                data = await asyncio.wait_for(
                    self._reader.readuntil(TERMINATOR), timeout or self.timeout
                )
            except (OSError, asyncio.IncompleteReadError, asyncio.TimeoutError) as err:
                _LOGGER.warning("Command %r failed (%s), reconnecting", cmd, err)
                await self.close()
                raise ConnectionError(f"{cmd} failed: {err}") from err
            text = data[: -len(TERMINATOR)].decode("ascii", errors="replace")
            if not echoes(text, cmd):
                # An answer to an earlier command (after a timeout, say): every
                # answer from here on would be one behind, so start over clean
                _LOGGER.warning("Answer to %r doesn't start with its echo, resyncing", cmd)
                await self.close()
                raise ConnectionError(f"{cmd}: out of sync with the robot")
            await asyncio.sleep(INTER_COMMAND_DELAY_S)
            return strip_echo(text, cmd)
