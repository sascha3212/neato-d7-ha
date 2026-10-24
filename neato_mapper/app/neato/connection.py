"""Talks to the robot through the ESP32 serial bridge (raw serial over TCP)."""

from __future__ import annotations

import asyncio
import logging

from .protocol import TERMINATOR, echoes, strip_echo

_LOGGER = logging.getLogger(__name__)

# Pause between commands; the robot's serial task can hang when it is
# hammered back to back while it cleans
INTER_COMMAND_DELAY_S = 0.15
# After an answer that belongs to another command, wait for this much silence
RESYNC_QUIET_S = 0.5


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

    async def _drain_input(self, quiet_s: float = 0.3) -> None:
        """Throw away boot logs or half answers left in the pipe."""
        if self._reader is None:
            return
        while True:
            try:
                data = await asyncio.wait_for(self._reader.read(4096), quiet_s)
            except asyncio.TimeoutError:
                return
            if not data:
                return

    async def command(self, cmd: str, timeout: float | None = None) -> str:
        """Send one command and return its answer without the echo line."""
        async with self._lock:
            for attempt in range(2):
                text = await self._exchange(cmd, timeout)
                answer = self._match(text, cmd)
                if answer is not None:
                    await asyncio.sleep(INTER_COMMAND_DELAY_S)
                    return answer
                # Probably the tail of an earlier answer: let the pipe go
                # quiet, throw away what's in it, and ask again
                _LOGGER.warning("Answer to %r is for something else (%r), resyncing",
                                cmd, text.strip()[:120])
                await self._drain_input(quiet_s=RESYNC_QUIET_S)
            await self.close()
            raise ConnectionError(f"{cmd}: out of sync with the robot")

    async def send_only(self, cmd: str) -> None:
        """Send a command that gets no answer (the robot restarts on it)."""
        async with self._lock:
            if not self.connected:
                await self.connect()
            assert self._writer is not None
            try:
                self._writer.write(cmd.encode("ascii") + b"\n")
                await self._writer.drain()
            except OSError as err:
                await self.close()
                raise ConnectionError(f"{cmd} failed: {err}") from err
            await asyncio.sleep(INTER_COMMAND_DELAY_S)

    async def _exchange(self, cmd: str, timeout: float | None) -> str:
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
        return data[: -len(TERMINATOR)].decode("ascii", errors="replace")

    @staticmethod
    def _match(text: str, cmd: str) -> str | None:
        """The answer to cmd, or None if the text belongs to another command."""
        if echoes(text, cmd):
            return strip_echo(text, cmd)
        lines = text.replace("\r", "").split("\n")
        # Leftovers of an earlier answer in front of ours: our echo comes later
        for i, line in enumerate(lines):
            if line.strip().lower() == cmd.strip().lower():
                return "\n".join(lines[i + 1:])
        return None
