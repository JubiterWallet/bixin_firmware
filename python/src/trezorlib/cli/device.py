# This file is part of the Trezor project.
#
# Copyright (C) 2012-2022 SatoshiLabs and contributors
#
# This library is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License version 3
# as published by the Free Software Foundation.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the License along with this library.
# If not, see <https://www.gnu.org/licenses/lgpl-3.0.html>.

import sys
from typing import TYPE_CHECKING, Optional, Sequence

import click

from .. import debuglink, device, exceptions, messages, ui
from . import ChoiceType, with_client

if TYPE_CHECKING:
    from ..client import TrezorClient
    from . import TrezorConnection
    from ..protobuf import MessageType

RECOVERY_TYPE = {
    "scrambled": messages.RecoveryDeviceType.ScrambledWords,
    "matrix": messages.RecoveryDeviceType.Matrix,
}

BACKUP_TYPE = {
    "single": messages.BackupType.Bip39,
    "shamir": messages.BackupType.Slip39_Basic,
    "advanced": messages.BackupType.Slip39_Advanced,
}

SD_PROTECT_OPERATIONS = {
    "on": messages.SdProtectOperationType.ENABLE,
    "off": messages.SdProtectOperationType.DISABLE,
    "refresh": messages.SdProtectOperationType.REFRESH,
}

RESOURCE_UPLOAD_PURPOSE = {
    "wallpaper": messages.ResourceType.WallPaper,
    "nft": messages.ResourceType.Nft,
}


@click.group(name="device")
def cli() -> None:
    """Device management commands - setup, recover seed, wipe, etc."""


@cli.command()
@with_client
def self_test(client: "TrezorClient") -> str:
    """Perform a self-test."""
    return debuglink.self_test(client)


@cli.command()
@click.option(
    "-b",
    "--bootloader",
    help="Wipe device in bootloader mode. This also erases the firmware.",
    is_flag=True,
)
@with_client
def wipe(client: "TrezorClient", bootloader: bool) -> str:
    """Reset device to factory defaults and remove all private data."""
    if bootloader:
        if not client.features.bootloader_mode:
            click.echo("Please switch your device to bootloader mode.")
            sys.exit(1)
        else:
            click.echo("Wiping user data and firmware!")
    else:
        if client.features.bootloader_mode:
            click.echo(
                "Your device is in bootloader mode. This operation would also erase firmware."
            )
            click.echo(
                'Specify "--bootloader" if that is what you want, or disconnect and reconnect device in normal mode.'
            )
            click.echo("Aborting.")
            sys.exit(1)
        else:
            click.echo("Wiping user data!")

    try:
        return device.wipe(client)
    except exceptions.TrezorFailure as e:
        click.echo("Action failed: {} {}".format(*e.args))
        sys.exit(3)


@cli.command()
@click.option("-m", "--mnemonic", multiple=True)
@click.option("-p", "--pin", default="")
@click.option("-r", "--passphrase-protection", is_flag=True)
@click.option("-l", "--label", default="")
@click.option("-i", "--ignore-checksum", is_flag=True)
@click.option("-s", "--slip0014", is_flag=True)
@click.option("-b", "--needs-backup", is_flag=True)
@click.option("-n", "--no-backup", is_flag=True)
@with_client
def load(
    client: "TrezorClient",
    mnemonic: Sequence[str],
    pin: str,
    passphrase_protection: bool,
    label: str,
    ignore_checksum: bool,
    slip0014: bool,
    needs_backup: bool,
    no_backup: bool,
) -> str:
    """Upload seed and custom configuration to the device.

    This functionality is only available in debug mode.
    """
    if slip0014 and mnemonic:
        raise click.ClickException("Cannot use -s and -m together.")

    if slip0014:
        mnemonic = [" ".join(["all"] * 12)]
        if not label:
            label = "SLIP-0014"

    return debuglink.load_device(
        client,
        mnemonic=list(mnemonic),
        pin=pin,
        passphrase_protection=passphrase_protection,
        label=label,
        language="en-US",
        skip_checksum=ignore_checksum,
        needs_backup=needs_backup,
        no_backup=no_backup,
    )


@cli.command()
@click.option("-w", "--words", type=click.Choice(["12", "18", "24"]), default="24")
@click.option("-e", "--expand", is_flag=True)
@click.option("-p", "--pin-protection", is_flag=True)
@click.option("-r", "--passphrase-protection", is_flag=True)
@click.option("-l", "--label")
@click.option("-u", "--u2f-counter", default=None, type=int)
@click.option(
    "-t", "--type", "rec_type", type=ChoiceType(RECOVERY_TYPE), default="scrambled"
)
@click.option("-d", "--dry-run", is_flag=True)
@with_client
def recover(
    client: "TrezorClient",
    words: str,
    expand: bool,
    pin_protection: bool,
    passphrase_protection: bool,
    label: Optional[str],
    u2f_counter: int,
    rec_type: messages.RecoveryDeviceType,
    dry_run: bool,
) -> "MessageType":
    """Start safe recovery workflow."""
    if rec_type == messages.RecoveryDeviceType.ScrambledWords:
        input_callback = ui.mnemonic_words(expand)
    else:
        input_callback = ui.matrix_words
        click.echo(ui.RECOVERY_MATRIX_DESCRIPTION)

    return device.recover(
        client,
        word_count=int(words),
        passphrase_protection=passphrase_protection,
        pin_protection=pin_protection,
        label=label,
        u2f_counter=u2f_counter,
        language="en-US",
        input_callback=input_callback,
        type=rec_type,
        dry_run=dry_run,
    )


@cli.command()
@click.option("-e", "--show-entropy", is_flag=True)
@click.option("-t", "--strength", type=click.Choice(["128", "192", "256"]))
@click.option("-r", "--passphrase-protection", is_flag=True)
@click.option("-p", "--pin-protection", is_flag=True)
@click.option("-l", "--label")
@click.option("-u", "--u2f-counter", default=0)
@click.option("-s", "--skip-backup", is_flag=True)
@click.option("-n", "--no-backup", is_flag=True)
@click.option("-b", "--backup-type", type=ChoiceType(BACKUP_TYPE), default="single")
@with_client
def setup(
    client: "TrezorClient",
    show_entropy: bool,
    strength: Optional[int],
    passphrase_protection: bool,
    pin_protection: bool,
    label: Optional[str],
    u2f_counter: int,
    skip_backup: bool,
    no_backup: bool,
    backup_type: messages.BackupType,
) -> str:
    """Perform device setup and generate new seed."""
    if strength:
        strength = int(strength)

    if (
        backup_type == messages.BackupType.Slip39_Basic
        and messages.Capability.Shamir not in client.features.capabilities
    ) or (
        backup_type == messages.BackupType.Slip39_Advanced
        and messages.Capability.ShamirGroups not in client.features.capabilities
    ):
        click.echo(
            "WARNING: Your Trezor device does not indicate support for the requested\n"
            "backup type. Traditional single-seed backup may be generated instead."
        )

    return device.reset(
        client,
        display_random=show_entropy,
        strength=strength,
        passphrase_protection=passphrase_protection,
        pin_protection=pin_protection,
        label=label,
        language="en-US",
        u2f_counter=u2f_counter,
        skip_backup=skip_backup,
        no_backup=no_backup,
        backup_type=backup_type,
    )


@cli.command()
@with_client
def backup(client: "TrezorClient") -> str:
    """Perform device seed backup."""
    return device.backup(client)


@cli.command()
@click.argument("operation", type=ChoiceType(SD_PROTECT_OPERATIONS))
@with_client
def sd_protect(
    client: "TrezorClient", operation: messages.SdProtectOperationType
) -> str:
    """Secure the device with SD card protection.

    When SD card protection is enabled, a randomly generated secret is stored
    on the SD card. During every PIN checking and unlocking operation this
    secret is combined with the entered PIN value to decrypt data stored on
    the device. The SD card will thus be needed every time you unlock the
    device. The options are:

    \b
    on - Generate SD card secret and use it to protect the PIN and storage.
    off - Remove SD card secret protection.
    refresh - Replace the current SD card secret with a new one.
    """
    if client.features.model == "1":
        raise click.ClickException("Trezor One does not support SD card protection.")
    return device.sd_protect(client, operation)


@cli.command()
@click.pass_obj
def reboot_to_bootloader(obj: "TrezorConnection") -> str:
    """Reboot device into bootloader mode.
    """
    # avoid using @with_client because it closes the session afterwards,
    # which triggers double prompt on device
    with obj.client_context() as client:
        return device.reboot(client)

@cli.command()
@click.pass_obj
def reboot_to_boardloader(obj: "TrezorConnection") -> str:
    """Reboot device into boardloader mode.

    Currently only supported on Trezor Model T.
    """
    # avoid using @with_client because it closes the session afterwards,
    # which triggers double prompt on device
    with obj.client_context() as client:
        if client.features.model != "T":
            click.echo(f"Reboot to boardloader is not support on OneKey {client.features.model}")
        return device.reboot(client, False)


@cli.command()
@click.pass_obj
def se_read_cert(obj: "TrezorConnection") -> bytes:
    """Get device se cert.

    Used in device verify.
    """
    with obj.client_context() as client:
        cert_bytes = device.se_read_cert(client).public_cert
        return cert_bytes


@cli.command()
# fmt: off
@click.option("-f", "--fullpath", help="The full path of the file to upload")
@click.option("-z", "--zoompath", help="The zoom file of the image to upload")
@click.option("-p", "--purpose", type=ChoiceType(RESOURCE_UPLOAD_PURPOSE), default="wallpaper", help="The upload file used for")
@click.option("-m", "--metadata", help="the metadata of the nft, a json string include header, subheader, network and owner fields")
# fmt: on
@with_client
def upload_res(
    client: "TrezorClient",
    fullpath: str,
    zoompath: str,
    purpose: int,
    metadata: str
) -> None:
    """Upload wallpaper/nft to device."""
    if fullpath:
        ext = fullpath.split(".")[-1]
        with open(zoompath, "rb") as f:
            zoomdata = f.read()
        with open(fullpath, "rb") as f:
            data = f.read()
        try:
            click.echo("Please confirm the action on your Trezor device")

            click.echo("Uploading...\r", nl=False)
            with click.progressbar(
                label="Uploading", length=len(data) + len(zoomdata), show_eta=False
            ) as bar:
                device.upload_res(
                    client,
                    ext,
                    data,
                    zoomdata,
                    progress_update=bar.update,
                    res_type=purpose,
                    nft_metadata=metadata if metadata else None,
                )
        except exceptions.Cancelled:
            click.echo("Upload aborted on device.")
        except exceptions.TrezorException as e:
            click.echo(f"Upload failed: {e}")
            sys.exit(3)


@cli.command()
# fmt: off
@click.option("-f", "--fullpath", help="The full path of the file to update")
# fmt: on
@with_client
def update_res(
    client: "TrezorClient",
    fullpath: str,
) -> None:
    """Update internal static resource(internal icons)."""
    if fullpath:
        file_name = fullpath.split("/")[-1]
        with open(fullpath, "rb") as f:
            data = f.read()
        try:
            click.echo("Uploading...\r", nl=False)
            with click.progressbar(
                label="Uploading", length=len(data), show_eta=False
            ) as bar:
                device.update_res(client, file_name, data, progress_update=bar.update)
        except exceptions.Cancelled:
            click.echo("Upload aborted on device.")
        except exceptions.TrezorException as e:
            click.echo(f"Update failed: {e}")
            sys.exit(3)

@cli.command()
# fmt: off
@click.option("-p", "--path_dir", help="The path of dir to enum")
# fmt: on
@with_client
def list_dir(client: "TrezorClient", path_dir: str) -> None:
    files_info = device.list_dir(client, path_dir)
    for info in files_info:
        click.echo(f"file_name {info.name} with size {info.size} bytes")
