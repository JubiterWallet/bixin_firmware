from trezor import io, log, loop, utils, wire
from trezor.lvglui import lvgl_tick
from trezor.uart import handle_ble_info, handle_uart, handle_usb_state

import apps.base
import usb

apps.base.boot()

if not utils.BITCOIN_ONLY and usb.ENABLE_IFACE_WEBAUTHN:
    import apps.webauthn

    apps.webauthn.boot()

if __debug__:
    import apps.debug

    apps.debug.boot()

# run main event loop and specify which screen is the default
apps.base.set_homescreen()

loop.schedule(handle_uart())

loop.schedule(handle_ble_info())

loop.schedule(handle_usb_state())

loop.schedule(lvgl_tick())

# initialize the wire codec
wire.setup(usb.iface_wire)
if __debug__:
    wire.setup(usb.iface_debug, is_debug_session=True)
# interface used for trezor wire protocol
SPI_IFACE_NUM = 6
wire.setup(
    io.SPI(
        SPI_IFACE_NUM,
    )
)
if utils.show_app_guide():
    from trezor.ui.layouts import show_onekey_app_guide

    loop.schedule(show_onekey_app_guide())

loop.run()

if __debug__:
    log.debug(__name__, "Restarting main loop")
