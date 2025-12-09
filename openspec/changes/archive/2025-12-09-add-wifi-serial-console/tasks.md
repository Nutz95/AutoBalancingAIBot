---
change-id: add-wifi-serial-console
---

# Tasks for add-wifi-serial-console

Checklist (update after implementation)

- [x] Update documentation: expand `ESP32/README.md` with serial Wi‑Fi configuration steps and Python client usage.
- [x] Add OpenSpec proposal and tasks (`proposal.md`, `design.md`, `tasks.md`).
- [x] Verify Python client handles disconnect/reconnect/resend (manual validation performed).
- [ ] Optional: implement OS-specific TCP keepalive tuning (Windows) to reduce detection delay.
- [ ] Optional: add authentication token support for TCP console (requires firmware changes and new serial commands).

Notes:
- The first three items are completed: README updated, OpenSpec files added, and the example Python client was tested and confirmed to reconnect and resend failed commands during a device flash.
- Remaining items are optional improvements that can be scheduled separately if desired.
