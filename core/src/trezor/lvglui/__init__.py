from storage import device
from trezor import io, log, loop, utils

import lvgl as lv  # type: ignore[Import "lvgl" could not be resolved]


async def lvgl_tick():
    from trezor import workflow

    inactive_time_bak = 0
    while True:
        if utils.EMULATOR:
            lv.tick_inc(10)
        await loop.sleep(10)
        lv.timer_handler()
        inactive_time = get_elapsed()
        if inactive_time < inactive_time_bak:
            workflow.idle_timer.touch()

        inactive_time_bak = inactive_time


def init_lvgl() -> None:
    import lvgldrv as lcd  # type: ignore[Import "lvgldrv" could not be resolved]

    lv.init()
    if not utils.EMULATOR:
        import stjpeg  # type: ignore[Import "stjpeg" could not be resolved]

        stjpeg.init()
    disp_buf1 = lv.disp_draw_buf_t()
    buf1_1 = lcd.framebuffer(1)
    disp_buf1.init(buf1_1, None, len(buf1_1) // lv.color_t.__SIZE__)
    disp_drv = lv.disp_drv_t()
    disp_drv.init()
    disp_drv.draw_buf = disp_buf1
    disp_drv.flush_cb = lcd.flush
    disp_drv.hor_res = 480
    disp_drv.ver_res = 800
    disp_drv.register()

    indev_drv = lv.indev_drv_t()
    indev_drv.init()
    indev_drv.type = lv.INDEV_TYPE.POINTER
    indev_drv.read_cb = lcd.ts_read
    indev_drv.register()


def init_theme() -> None:
    dispp = lv.disp_get_default()
    theme = lv.theme_default_init(
        dispp,
        lv.palette_main(lv.PALETTE.BLUE),
        lv.palette_main(lv.PALETTE.RED),
        True,
        lv.font_default(),
    )
    # dispp.set_bg_color(lv.color_hex(0x000000))
    dispp.set_theme(theme)


def init_file_system() -> None:
    if not utils.EMULATOR:

        def clean():
            for size, attrs, name in io.fatfs.listdir("1:/res/wallpapers"):
                if size > 0 and attrs[1] == "h":
                    io.fatfs.unlink(f"1:/res/wallpapers/{name}")
            for size, attrs, name in io.fatfs.listdir("1:/res/nfts/imgs"):
                if size > 0 and attrs[1] == "h":
                    io.fatfs.unlink(f"1:/res/nfts/imgs/{name}")
            for size, attrs, name in io.fatfs.listdir("1:/res/nfts/zooms"):
                if size > 0 and (attrs[1] == "h" or name[:1] == "."):
                    io.fatfs.unlink(f"1:/res/nfts/zooms/{name}")
            for size, attrs, name in io.fatfs.listdir("1:/res/nfts/desc"):
                if size > 0 and (attrs[1] == "h" or name[:1] == "."):
                    io.fatfs.unlink(f"1:/res/nfts/desc/{name}")

        io.fatfs.mount()
        io.fatfs.mkdir("1:/res", True)
        io.fatfs.mkdir("1:/res/wallpapers", True)
        io.fatfs.mkdir("1:/res/nfts", True)
        io.fatfs.mkdir("1:/res/nfts/imgs", True)
        io.fatfs.mkdir("1:/res/nfts/zooms", True)
        io.fatfs.mkdir("1:/res/nfts/desc", True)
        clean()


def get_elapsed() -> int:
    """Get elapsed time since last user activity(e.g. click).
    @return: elapsed time in milliseconds
    """
    disp = lv.disp_get_default()
    return disp.get_inactive_time()


try:
    init_file_system()
    init_lvgl()
    init_theme()
    if __debug__:
        log.info("init", "initialized successfully")
except BaseException:
    if __debug__:
        log.error("init", "failed to initialize emulator")


class StatusBar(lv.obj):
    _instance = None

    BLE_STATE_CONNECTED = 0
    BLE_STATE_DISABLED = 1
    BLE_STATE_ENABLED = 2

    @classmethod
    def get_instance(cls) -> "StatusBar":
        if cls._instance is None:
            cls._instance = StatusBar()
        return cls._instance

    def __init__(self):
        super().__init__(lv.layer_top())
        self.set_size(lv.pct(80), lv.SIZE.CONTENT)
        self.align(lv.ALIGN.TOP_RIGHT, 0, 0)
        self.set_style_border_width(0, 0)
        self.set_style_bg_opa(0, 0)
        self.set_style_pad_all(0, 0)
        self.set_style_pad_column(0, 0)
        self.set_style_pad_right(8, 0)
        self.set_style_pad_top(6, 0)
        self.set_flex_flow(lv.FLEX_FLOW.ROW)
        # align style of the items in the container
        self.set_flex_align(
            lv.FLEX_ALIGN.END, lv.FLEX_ALIGN.CENTER, lv.FLEX_ALIGN.CENTER
        )
        # usb status
        self.usb = lv.img(self)
        self.usb.set_src("A:/res/usb.png")
        self.usb.add_flag(lv.obj.FLAG.HIDDEN)

        # ble status
        ble_enabled = device.ble_enabled()
        self.ble = lv.img(self)
        self.ble.set_src(
            "A:/res/ble-enabled.png" if ble_enabled else "A:/res/ble-disabled.png"
        )

        # battery capacity percent
        self.percent = lv.label(self)
        from trezor.lvglui.scrs import font_STATUS_BAR

        self.percent.set_style_text_font(font_STATUS_BAR, 0)
        self.percent.set_style_text_letter_space(-1, 0)
        self.percent.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
        self.percent.set_style_pad_hor(5, 0)
        self.percent.set_style_pad_ver(4, 0)
        self.percent.add_flag(lv.obj.FLAG.HIDDEN)

        # battery capacity icon
        self.battery = lv.img(self)
        self.battery.set_src("A:/res/battery-60-white.png")
        self.battery.add_flag(lv.obj.FLAG.HIDDEN)
        # charging status
        self.charging = lv.img(self)
        self.charging.set_src("A:/res/charging.png")
        self.charging.add_flag(lv.obj.FLAG.HIDDEN)

    def show_ble(self, status: int):
        if status == StatusBar.BLE_STATE_CONNECTED:
            icon_path = "A:/res/ble-connected.png"
        elif status == StatusBar.BLE_STATE_ENABLED:
            icon_path = "A:/res/ble-enabled.png"
        else:
            icon_path = "A:/res/ble-disabled.png"
        self.ble.set_src(icon_path)

    def show_usb(self, show: bool = False):
        if show:
            if self.usb.has_flag(lv.obj.FLAG.HIDDEN):
                self.usb.clear_flag(lv.obj.FLAG.HIDDEN)
        else:
            if not self.usb.has_flag(lv.obj.FLAG.HIDDEN):
                self.usb.add_flag(lv.obj.FLAG.HIDDEN)

    def show_charging(self, show: bool = False):
        if show:
            if self.charging.has_flag(lv.obj.FLAG.HIDDEN):
                self.charging.clear_flag(lv.obj.FLAG.HIDDEN)
        else:
            if not self.charging.has_flag(lv.obj.FLAG.HIDDEN):
                self.charging.add_flag(lv.obj.FLAG.HIDDEN)

    def set_battery_img(self, value: int, charging: bool):
        if charging:
            self.percent.clear_flag(lv.obj.FLAG.HIDDEN)
            self.percent.set_text(f"{min(value, 100)}%")
        else:
            self.percent.add_flag(lv.obj.FLAG.HIDDEN)
        icon_path = retrieve_icon_path(value, charging)
        self.battery.clear_flag(lv.obj.FLAG.HIDDEN)
        self.battery.set_src(icon_path)


def retrieve_icon_path(value: int, charging: bool) -> str:

    if value >= 95:
        return (
            "A:/res/battery-100-green.png"
            if charging
            else "A:/res/battery-100-white.png"
        )
    elif value >= 90:
        return (
            "A:/res/battery-95-green.png" if charging else "A:/res/battery-95-white.png"
        )
    elif value >= 85:
        return (
            "A:/res/battery-90-green.png" if charging else "A:/res/battery-90-white.png"
        )
    elif value >= 80:
        return (
            "A:/res/battery-85-green.png" if charging else "A:/res/battery-85-white.png"
        )
    elif value >= 75:
        return (
            "A:/res/battery-80-green.png" if charging else "A:/res/battery-80-white.png"
        )
    elif value >= 70:
        return (
            "A:/res/battery-75-green.png" if charging else "A:/res/battery-75-white.png"
        )
    elif value >= 65:
        return (
            "A:/res/battery-70-green.png" if charging else "A:/res/battery-70-white.png"
        )
    elif value >= 60:
        return (
            "A:/res/battery-65-green.png" if charging else "A:/res/battery-65-white.png"
        )
    elif value >= 55:
        return (
            "A:/res/battery-60-green.png" if charging else "A:/res/battery-60-white.png"
        )
    elif value >= 50:
        return (
            "A:/res/battery-55-green.png" if charging else "A:/res/battery-55-white.png"
        )
    elif value >= 45:
        return (
            "A:/res/battery-50-green.png" if charging else "A:/res/battery-50-white.png"
        )
    elif value >= 40:
        return (
            "A:/res/battery-45-green.png" if charging else "A:/res/battery-45-white.png"
        )
    elif value >= 35:
        return (
            "A:/res/battery-40-green.png" if charging else "A:/res/battery-40-white.png"
        )
    elif value >= 30:
        return (
            "A:/res/battery-35-green.png" if charging else "A:/res/battery-35-white.png"
        )
    elif value >= 25:
        return (
            "A:/res/battery-30-green.png" if charging else "A:/res/battery-30-white.png"
        )
    elif value >= 20:
        return (
            "A:/res/battery-25-green.png" if charging else "A:/res/battery-25-white.png"
        )
    elif value >= 15:
        return (
            "A:/res/battery-20-green.png" if charging else "A:/res/battery-20-white.png"
        )
    elif value >= 10:
        return (
            "A:/res/battery-15-green.png" if charging else "A:/res/battery-15-white.png"
        )
    elif value >= 5:
        return (
            "A:/res/battery-10-green.png" if charging else "A:/res/battery-10-white.png"
        )
    elif value >= 0:
        return (
            "A:/res/battery-5-green.png" if charging else "A:/res/battery-5-white.png"
        )
    else:
        # show nothing, the path is not exist
        return (
            "A:/res/battery-none-green.png"
            if charging
            else "A:/res/battery-none-white.png"
        )
