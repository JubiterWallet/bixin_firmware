from typing import TYPE_CHECKING

import storage.cache
import storage.device
from trezor import config, ui, utils, wire, workflow
from trezor.enums import MessageType
from trezor.messages import Success

from . import workflow_handlers

if TYPE_CHECKING:
    from trezor import protobuf
    from trezor.messages import (
        Features,
        Initialize,
        EndSession,
        GetFeatures,
        Cancel,
        LockDevice,
        Ping,
        DoPreauthorized,
        CancelAuthorization,
    )


def get_features() -> Features:
    import storage.recovery
    import storage.sd_salt
    import storage  # workaround for https://github.com/microsoft/pyright/issues/2685

    from trezor import sdcard
    from trezor.enums import Capability
    from trezor.messages import Features
    from trezor import uart
    from apps.common import mnemonic, safety_checks

    f = Features(
        vendor="onekey.so",
        language=storage.device.get_language(),
        major_version=utils.VERSION_MAJOR,
        minor_version=utils.VERSION_MINOR,
        patch_version=utils.VERSION_PATCH,
        onekey_version=utils.ONEKEY_VERSION,
        revision=utils.SCM_REVISION,
        model=utils.MODEL,
        device_id=storage.device.get_device_id(),
        label=storage.device.get_label(),
        pin_protection=config.has_pin(),
        unlocked=config.is_unlocked(),
        ble_name=uart.get_ble_name(),
        ble_ver=uart.get_ble_version(),
        ble_enable=storage.device.ble_enabled(),
        serial_no=storage.device.get_serial(),
        build_id=utils.BUILD_ID,
        bootloader_version=utils.boot_version(),
        boardloader_version=utils.board_version(),
    )

    if utils.BITCOIN_ONLY:
        f.capabilities = [
            Capability.Bitcoin,
            Capability.Crypto,
            Capability.Shamir,
            Capability.ShamirGroups,
            Capability.PassphraseEntry,
        ]
    else:
        f.capabilities = [
            Capability.Bitcoin,
            Capability.Bitcoin_like,
            Capability.Binance,
            Capability.Cardano,
            Capability.Crypto,
            Capability.EOS,
            Capability.Ethereum,
            Capability.Monero,
            Capability.NEM,
            Capability.Ripple,
            Capability.Stellar,
            Capability.Tezos,
            Capability.U2F,
            Capability.Shamir,
            Capability.ShamirGroups,
            Capability.PassphraseEntry,
        ]
    f.sd_card_present = sdcard.is_present()
    f.initialized = storage.device.is_initialized()

    # private fields:
    if config.is_unlocked():
        # passphrase_protection is private, see #1807
        f.passphrase_protection = storage.device.is_passphrase_enabled()
        f.needs_backup = storage.device.needs_backup()
        f.unfinished_backup = storage.device.unfinished_backup()
        f.no_backup = storage.device.no_backup()
        f.flags = storage.device.get_flags()
        f.recovery_mode = storage.recovery.is_in_progress()
        f.backup_type = mnemonic.get_type()
        f.sd_protection = storage.sd_salt.is_enabled()
        f.wipe_code_protection = config.has_wipe_code()
        f.passphrase_always_on_device = storage.device.get_passphrase_always_on_device()
        f.safety_checks = safety_checks.read_setting()
        f.auto_lock_delay_ms = storage.device.get_autolock_delay_ms()
        f.display_rotation = storage.device.get_rotation()
        f.experimental_features = storage.device.get_experimental_features()

    return f


async def handle_Initialize(ctx: wire.Context, msg: Initialize) -> Features:
    session_id = storage.cache.start_session(msg.session_id)

    if not utils.BITCOIN_ONLY:
        derive_cardano = storage.cache.get(storage.cache.APP_COMMON_DERIVE_CARDANO)
        have_seed = storage.cache.is_set(storage.cache.APP_COMMON_SEED)

        if (
            have_seed
            and msg.derive_cardano is not None
            and msg.derive_cardano != bool(derive_cardano)
        ):
            # seed is already derived, and host wants to change derive_cardano setting
            # => create a new session
            storage.cache.end_current_session()
            session_id = storage.cache.start_session()
            have_seed = False

        if not have_seed:
            storage.cache.set(
                storage.cache.APP_COMMON_DERIVE_CARDANO,
                b"\x01" if msg.derive_cardano else b"",
            )

    features = get_features()
    features.session_id = session_id
    storage.cache.update_res_confirm_refresh()
    return features


async def handle_GetFeatures(ctx: wire.Context, msg: GetFeatures) -> Features:
    return get_features()


async def handle_Cancel(ctx: wire.Context, msg: Cancel) -> Success:
    raise wire.ActionCancelled


async def handle_LockDevice(ctx: wire.Context, msg: LockDevice) -> Success:
    lock_device()
    return Success()


async def handle_EndSession(ctx: wire.Context, msg: EndSession) -> Success:
    storage.cache.end_current_session()
    return Success()


async def handle_Ping(ctx: wire.Context, msg: Ping) -> Success:
    if msg.button_protection:
        from trezor.ui.layouts import confirm_action
        from trezor.enums import ButtonRequestType as B

        await confirm_action(ctx, "ping", "Confirm", "ping", br_code=B.ProtectCall)
    return Success(message=msg.message)


async def handle_DoPreauthorized(
    ctx: wire.Context, msg: DoPreauthorized
) -> protobuf.MessageType:
    from trezor.messages import PreauthorizedRequest
    from apps.common import authorization

    if not authorization.is_set():
        raise wire.ProcessError("No preauthorized operation")

    wire_types = authorization.get_wire_types()
    utils.ensure(bool(wire_types), "Unsupported preauthorization found")

    req = await ctx.call_any(PreauthorizedRequest(), *wire_types)

    assert req.MESSAGE_WIRE_TYPE is not None
    handler = workflow_handlers.find_registered_handler(
        ctx.iface, req.MESSAGE_WIRE_TYPE
    )
    if handler is None:
        return wire.unexpected_message()

    return await handler(ctx, req, authorization.get())  # type: ignore [Expected 2 positional arguments]


async def handle_CancelAuthorization(
    ctx: wire.Context, msg: CancelAuthorization
) -> protobuf.MessageType:
    from apps.common import authorization

    authorization.clear()
    return Success(message="Authorization cancelled")


ALLOW_WHILE_LOCKED = (
    MessageType.Initialize,
    MessageType.EndSession,
    MessageType.GetFeatures,
    MessageType.Cancel,
    MessageType.LockDevice,
    MessageType.DoPreauthorized,
    MessageType.WipeDevice,
)


def set_homescreen() -> None:
    import lvgl as lv  # type: ignore[Import "lvgl" could not be resolved]

    ble_name = storage.device.get_ble_name()
    if storage.device.is_initialized():
        dev_state = get_state()
        device_name = storage.device.get_label()
        if not config.is_unlocked():
            if __debug__:
                print("Device is locked")
            from trezor.lvglui.scrs.lockscreen import LockScreen

            screen = LockScreen(device_name, ble_name, dev_state)
        else:
            if __debug__:
                print("Device is unlocked")
            from trezor.lvglui.scrs.homescreen import MainScreen

            store_ble_name(ble_name)
            screen = MainScreen(device_name, ble_name, dev_state)
    else:
        from trezor.lvglui.scrs.initscreen import InitScreen

        InitScreen()
        return
    if not screen.is_visible():
        lv.scr_load(screen)


def store_ble_name(ble_name):
    from trezor import uart

    temp_ble_name = uart.get_ble_name()
    if not ble_name and temp_ble_name:
        storage.device.set_ble_name(temp_ble_name)


def get_state() -> str | None:
    from trezor.lvglui.i18n import gettext as _, keys as i18n_keys

    if storage.device.no_backup():
        dev_state = _(i18n_keys.MSG__SEEDLESS)
    elif storage.device.unfinished_backup():
        dev_state = _(i18n_keys.MSG__BACKUP_FAILED)
    elif storage.device.needs_backup():
        dev_state = _(i18n_keys.MSG__NEEDS_BACKUP)
    elif not config.has_pin():
        dev_state = _(i18n_keys.MSG__PIN_NOT_SET)
    elif storage.device.get_experimental_features():
        dev_state = _(i18n_keys.MSG__EXPERIMENTAL_MODE)
    else:
        dev_state = None
    return dev_state


def lock_device() -> None:
    if storage.device.is_initialized() and config.has_pin():
        config.lock()
        wire.find_handler = get_pinlocked_handler
        set_homescreen()
        workflow.close_others()


def lock_device_if_unlocked() -> None:
    if config.is_unlocked():
        lock_device()
    from trezor import loop

    loop.schedule(utils.turn_off_lcd())


def screen_off_if_possible() -> None:
    if not ui.display.backlight():
        return

    if ui.display.backlight():
        if config.is_unlocked():
            ui.display.backlight(ui.style.BACKLIGHT_LOW)
        workflow.idle_timer.set(3 * 1000, lock_device_if_unlocked)


def shutdown_device() -> None:
    from trezor import uart

    if storage.device.is_initialized():
        if not uart.CHARGING:
            uart.ctrl_power_off()


async def unlock_device(ctx: wire.GenericContext = wire.DUMMY_CONTEXT) -> None:
    """Ensure the device is in unlocked state.

    If the storage is locked, attempt to unlock it. Reset the homescreen and the wire
    handler.
    """
    from apps.common.request_pin import verify_user_pin

    if not config.is_unlocked():
        # verify_user_pin will raise if the PIN was invalid
        await verify_user_pin(ctx)
    # reset the idle_timer
    reload_settings_from_storage()
    set_homescreen()
    wire.find_handler = workflow_handlers.find_registered_handler


def get_pinlocked_handler(
    iface: wire.WireInterface, msg_type: int
) -> wire.Handler[wire.Msg] | None:
    orig_handler = workflow_handlers.find_registered_handler(iface, msg_type)
    if orig_handler is None:
        return None

    if __debug__:
        import usb

        if iface is usb.iface_debug:
            return orig_handler

    if msg_type in ALLOW_WHILE_LOCKED:
        return orig_handler

    async def wrapper(ctx: wire.Context, msg: wire.Msg) -> protobuf.MessageType:
        await unlock_device(ctx)
        return await orig_handler(ctx, msg)

    return wrapper


# this function is also called when handling ApplySettings
def reload_settings_from_storage(timeout_ms: int | None = None) -> None:
    if not storage.device.is_initialized():
        return
    workflow.idle_timer.remove(lock_device_if_unlocked)
    workflow.idle_timer.set(
        timeout_ms
        if timeout_ms is not None
        else storage.device.get_autolock_delay_ms(),
        screen_off_if_possible,
    )
    if utils.AUTO_POWER_OFF:
        workflow.idle_timer.set(
            storage.device.get_autoshutdown_delay_ms(), shutdown_device
        )
    else:
        workflow.idle_timer.remove(shutdown_device)
    wire.experimental_enabled = storage.device.get_experimental_features()
    ui.display.orientation(storage.device.get_rotation())


def boot() -> None:
    workflow_handlers.register(MessageType.Initialize, handle_Initialize)
    workflow_handlers.register(MessageType.GetFeatures, handle_GetFeatures)
    workflow_handlers.register(MessageType.Cancel, handle_Cancel)
    workflow_handlers.register(MessageType.LockDevice, handle_LockDevice)
    workflow_handlers.register(MessageType.EndSession, handle_EndSession)
    workflow_handlers.register(MessageType.Ping, handle_Ping)
    workflow_handlers.register(MessageType.DoPreauthorized, handle_DoPreauthorized)
    workflow_handlers.register(
        MessageType.CancelAuthorization, handle_CancelAuthorization
    )

    reload_settings_from_storage()
    if config.is_unlocked():
        wire.find_handler = workflow_handlers.find_registered_handler
    else:
        wire.find_handler = get_pinlocked_handler
