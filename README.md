# balancing-bot-v2

PlatformIO project scaffold for the second iteration of a self-balancing robot firmware.

## Status

The repository currently contains the base PlatformIO layout and is ready for board-specific configuration and firmware development.

## Project structure

```text
.
|-- include/         # Project header files
|-- lib/             # Private libraries
|-- src/             # Application source files
|-- test/            # PlatformIO unit tests
|-- platformio.ini   # PlatformIO project configuration
`-- LICENSE          # MIT license
```

## Getting started

1. Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) or use the PlatformIO IDE extension.
2. Update `platformio.ini` with the correct platform, board, framework, and any project-specific settings.
3. Add your firmware entry point under `src/` (for example `src/main.cpp`).

## Common commands

```bash
pio run               # Build firmware
pio run -t upload     # Upload firmware
pio test              # Run tests
```

## Notes

- `include/README`, `lib/README`, and `test/README` describe the intended purpose of those directories.
- No application source has been added yet, so the project needs a configured environment and initial firmware before it can be built successfully.

## License

Released under the [MIT License](./LICENSE).
