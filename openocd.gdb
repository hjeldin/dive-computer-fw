target extended-remote :3333
set print asm-demangle on
set backtrace limit 32


monitor arm semihosting enable
monitor reset halt
file ./target/thumbv7em-none-eabi/release/stm-blink
break sdmmc/mod.rs:1056
load
monitor reset

# start the process but immediately halt the processor
stepi
