# User preferences

- Work one step at a time. Propose a plan and wait for the user's command. An explicit request authorizes only that step; report completion and stop.
- Keep actions and documentation within the requested scope. Do not add architecture, motion flows, launch instructions, tests, or future steps unless requested.
- Keep Markdown clear and minimal. For a package overview, give a brief workspace description and one short role per package. Expand living documents only when asked.
- Edit Markdown files directly. Terminal commands may run directly; tmux is optional.
- Display build output live and save all stdout/stderr in root `build_logs/`: one master log per build and separate package/process logs. Preserve logs unless asked to clear them.
- Run installations interactively so the user can enter passwords and answer prompts. Never request passwords in chat; follow sandbox approval requirements.
