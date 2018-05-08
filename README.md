## Wunderbar NRF51822 master module

This software is adaptation of original [WunderBar firmware](https://github.com/relayr/WunderBar-firmware-release).

# What's needed:
- [ninja build](https://github.com/ninja-build/ninja)
- [ARM GCC](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
- [NRF SDK v6.1](https://developer.nordicsemi.com/nRF5_SDK/nRF51_SDK_v6.x.x/)
- [NRF S120 v1.0.1 Softdevice](https://www.nordicsemi.com/eng/nordic/Products/nRF51822/S120-SD-v1/29098)

# Setup:
Set correct paths to GCC, NRF SDK and NRF Softdevice in build.ninja
- `GCC_ARM = /path/to/arm/gcc/bin`
- `NORDIC_SDK_BASE = /path/to/main/SDK/folder`
- Softdevice folder and softdevice hex file should not have version in name (ie: `s120_nrf51822` / `s120_nrf51822_softdevice.hex`) and should be located under `$NORDIC_SDK_BASE`

# Build:
- `ninja flashsoftdevice` - needed only once, if JLink does not work try: `nrfjprog  --program ../$NORDIC_SDK_BASE/s120_nrf51822/s120_nrf51822_softdevice.hex --chiperase`
- `ninja flash` - updates the application

# License and copyright

Adaptations: Copyright (c) 2018 Slashdev SDG UG
Original copyright: Copyright (c) 2014 relayr

This software is licensed under MIT license.
