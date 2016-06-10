#!/usr/bin/env python3
#
# Copyright (C) 2016 Zubax Robotics <info@zubax.com>
#
# This program is free software: you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import sys
sys.path.insert(1, os.path.join(sys.path[0], 'pyuavcan'))

from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort, glob_one, download_newest, open_serial_port,\
    enforce, SerialCLI, catch, BackgroundSpinner, fatal, warning, BackgroundDelay, imperative, \
    load_firmware_via_gdb, convert_units_from_to
import logging
import time
import yaml
import binascii
import uavcan
from base64 import b64decode, b64encode
from contextlib import closing, contextmanager
from functools import partial


PRODUCT_NAME = 'io.px4.sapog'
DEFAULT_FIRMWARE_GLOB = 'https://files.zubax.com/products/%s/*.compound.bin' % PRODUCT_NAME
CAN_BITRATE = 125000
FLASH_OFFSET = 0x08000000
TOOLCHAIN_PREFIX = 'arm-none-eabi-'
DEBUGGER_PORT_GDB_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if00'
DEBUGGER_PORT_CLI_GLOB = '/dev/serial/by-id/*Black_Magic_Probe*-if02'
BOOT_TIMEOUT = 10
END_OF_BOOT_LOG_TIMEOUT = 3

TEMPERATURE_RANGE_DEGC = 10, 40
ESC_ERROR_LIMIT = 1000
STARTUP_DUTY_CYCLE = 0.001
STABILITY_TEST_DUTY_CYCLES = [0.3, 0.6]


logger = logging.getLogger('main')


args = init('''Production testing application for ESC based on PX4 Sapog open source firmware.
If you're a licensed manufacturer, you should have received usage
instructions with the manufacturing doc pack.''',
            lambda p: p.add_argument('iface', help='CAN interface or device path, e.g. "can0", "/dev/ttyACM0", etc.'),
            lambda p: p.add_argument('--firmware', '-f', help='location of the firmware file (if not provided, ' +
                                     'the firmware will be downloaded from Zubax Robotics file server)'),
            require_root=True)

info('''
Usage instructions:

1. Connect a CAN adapter to this computer. Supported adapters are:
1.1. SLCAN-compliant adapters. If you're using an SLCAN adapter,
     use its serial port name as CAN interface name (e.g. "/dev/ttyACM0").
1.2. SocketCAN-compatible adapters. In this case it is recommended to use
     8devices USB2CAN. Correct interface name would be "can0".

2. Connect exactly one DroneCode Probe to this computer.
   For more info refer to https://docs.zubax.com/dronecode_probe.

3. Follow the instructions printed in green. If you have any questions,
   don't hesitate to reach licensing@zubax.com, or use the emergency
   contacts provided to you earlier.
''')


def wait_for_boot():
    def handle_serial_port_hanging():
        fatal('DRWATSON HAS DETECTED A PROBLEM WITH CONNECTED HARDWARE AND NEEDS TO TERMINATE.\n'
              'A serial port operation has timed out. This usually indicates a problem with the connected '
              'hardware or its drivers. Please disconnect all USB devices currently connected to this computer, '
              "then connect them back and restart Drwatson. If you're using a virtual machine, please reboot it.",
              use_abort=True)

    with BackgroundDelay(BOOT_TIMEOUT * 5, handle_serial_port_hanging):
        with open_serial_port(DEBUGGER_PORT_CLI_GLOB) as p:
            try:
                serial_cli = SerialCLI(p)
                boot_deadline = time.monotonic() + BOOT_TIMEOUT
                boot_notification_received = False
                failure_notification_received = False

                while boot_deadline > time.monotonic():
                    timed_out, line = serial_cli.read_line(END_OF_BOOT_LOG_TIMEOUT)

                    if not timed_out:
                        logger.info('CLI output: %r', line)

                        if PRODUCT_NAME in line:
                            info('Boot confirmed')
                            boot_notification_received = True

                        if 'error' in line.lower() or 'fail' in line.lower():
                            failure_notification_received = True
                            warning('Boot error: %r', line)
                    else:
                        if failure_notification_received:
                            abort('Device failed to start up normally; see the log for details')

                        if boot_notification_received:
                            return

            except IOError:
                logging.info('Boot error', exc_info=True)
            finally:
                p.flushInput()

    warning("The board did not report to CLI with a correct boot message, but we're going "
            "to continue anyway. Possible reasons for this warning:\n"
            '1. The board could not boot properly (however it was flashed successfully).\n'
            '2. The debug connector is not soldered properly.\n'
            '3. The serial port is open by another application.\n'
            '4. Either USB-UART adapter or VM are malfunctioning. Try to re-connect the '
            'adapter (disconnect from USB and from the board!) or reboot the VM.')


def test_uavcan():
    node_info = uavcan.protocol.GetNodeInfo.Response()
    node_info.name = 'com.zubax.drwatson.sapog'

    iface = init_can_iface()

    with closing(uavcan.make_node(iface,
                                  bitrate=CAN_BITRATE,
                                  node_id=127,
                                  mode=uavcan.protocol.NodeStatus().MODE_OPERATIONAL,
                                  node_info=node_info)) as n:
        def safe_spin(timeout):
            try:
                n.spin(timeout)
            except uavcan.UAVCANException:
                logger.error('Node spin failure', exc_info=True)

        @contextmanager
        def time_limit(timeout, error_fmt, *args):
            aborter = n.defer(timeout, partial(abort, error_fmt, *args))
            yield
            aborter.remove()

        # Dynamic node ID allocation
        try:
            nsmon = uavcan.app.node_monitor.NodeMonitor(n)
            alloc = uavcan.app.dynamic_node_id.CentralizedServer(n, nsmon)

            with time_limit(10, 'The node did not show up in time. Check CAN interface and crystal oscillator.'):
                while True:
                    safe_spin(1)
                    target_nodes = list(nsmon.find_all(lambda e: e.info and e.info.name.decode() == PRODUCT_NAME))
                    if len(target_nodes) == 1:
                        break
                    if len(target_nodes) > 1:
                        abort('Expected to find exactly one target node, found more: %r', target_nodes)

            node_id = target_nodes[0].node_id
            info('Node %r initialized', node_id)
            for nd in target_nodes:
                logger.info('Discovered node %r', nd)

            def request(what, fire_and_forget=False):
                response_event = None

                def cb(e):
                    nonlocal response_event
                    if not e:
                        abort('Request has timed out: %r', what)
                    response_event = e

                if fire_and_forget:
                    n.request(what, node_id, lambda _: None)
                    safe_spin(0.1)
                else:
                    n.request(what, node_id, cb)
                    while response_event is None:
                        safe_spin(0.1)
                    return response_event.response

            # Starting the node and checking its self-reported diag outputs
            def wait_for_init():
                with time_limit(10, 'The node did not complete initialization in time'):
                    while True:
                        safe_spin(1)
                        if nsmon.exists(node_id) and nsmon.get(node_id).status.mode == \
                                uavcan.protocol.NodeStatus().MODE_OPERATIONAL:
                            break

            def check_status():
                status = nsmon.get(node_id).status
                enforce(status.mode == uavcan.protocol.NodeStatus().MODE_OPERATIONAL,
                        'Unexpected operating mode')
                enforce(status.health == uavcan.protocol.NodeStatus().HEALTH_OK,
                        'Bad node health')

            info('Waiting for the node to complete initialization...')
            wait_for_init()
            check_status()

            col_esc_status = uavcan.app.message_collector.MessageCollector(n, uavcan.equipment.esc.Status, timeout=2)

            def check_everything(check_rotation=False):
                check_status()

                try:
                    m = col_esc_status[node_id].message
                except KeyError:
                    abort('Rock is dead.')
                else:
                    if check_rotation:
                        enforce(m.rpm > 0 and m.power_rating_pct > 0, 'Could not start the motor')
                        enforce(m.current > 0, 'Current is not positive')

                    enforce(m.error_count < ESC_ERROR_LIMIT, 'High error count: %r', m.error_count)

                    temp_degc = convert_units_from_to(m.temperature, 'Kelvin', 'Celsius')
                    enforce(TEMPERATURE_RANGE_DEGC[0] <= temp_degc <= TEMPERATURE_RANGE_DEGC[1],
                            'Invalid temperature: %r degC', temp_degc)

            # Testing before the motor is started
            imperative('CAUTION: THE MOTOR WILL START IN 2 SECONDS, KEEP CLEAR')
            safe_spin(2)
            check_everything()

            # Starting the motor
            esc_raw_command_bitlen = \
                uavcan.get_uavcan_data_type(uavcan.get_fields(uavcan.equipment.esc.RawCommand())['cmd'])\
                    .value_type.bitlen  # SO EASY TO USE

            def do_publish(duty_cycle, check_rotation):
                command_value = int(duty_cycle * (2 ** (esc_raw_command_bitlen - 1)))
                n.broadcast(uavcan.equipment.esc.RawCommand(cmd=[command_value]))
                check_everything(check_rotation)

            info('Starting the motor')
            publisher = n.periodic(0.01, partial(do_publish, STARTUP_DUTY_CYCLE, False))
            safe_spin(5)
            publisher.remove()

            info('Checking stability...')
            for dc in STABILITY_TEST_DUTY_CYCLES:
                info('Setting duty cycle %d%%...', int(dc * 100))
                publisher = n.periodic(0.01, partial(do_publish, dc, True))
                safe_spin(5)
                publisher.remove()

            info('Stopping...')
            latest_status = col_esc_status[node_id].message
            safe_spin(1)
            check_everything()

            # Final results
            imperative('Validate the latest ESC status variables (units are SI):\n%s', uavcan.to_yaml(latest_status))
        except Exception:
            for nid in nsmon.get_all_node_id():
                print('Node state: %r' % nsmon.get(nid))
            raise


def read_zubax_id(cli):
    zubax_id_lines = cli.write_line_and_read_output_lines_until_timeout('zubax_id')
    zubax_id_lines_joined = '\n'.join(zubax_id_lines)
    try:
        zubax_id = yaml.load(zubax_id_lines_joined)
    except Exception:
        logger.info('Could not parse YAML: %r', zubax_id_lines_joined)
        raise
    logger.info('Zubax ID: %r', zubax_id)
    return zubax_id


def init_can_iface():
    if '/' not in args.iface:
        logger.debug('Using iface %r as SocketCAN', args.iface)
        execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                              args.iface, args.iface, CAN_BITRATE)
        return args.iface
    else:
        logger.debug('Using iface %r as SLCAN', args.iface)

        speed_code = {
            1000000: 8,
            500000: 6,
            250000: 5,
            125000: 4,
            100000: 3
        }[CAN_BITRATE]

        execute_shell_command('killall -INT slcand &> /dev/null', ignore_failure=True)
        time.sleep(1)

        tty = os.path.realpath(args.iface).replace('/dev/', '')
        logger.debug('TTY %r', tty)

        execute_shell_command('slcan_attach -f -o -s%d /dev/%s', speed_code, tty)
        execute_shell_command('slcand %s', tty)

        iface_name = 'slcan0'
        time.sleep(1)
        execute_shell_command('ifconfig %s up', iface_name)
        execute_shell_command('ifconfig %s txqueuelen 1000', iface_name)

        return iface_name


def check_interfaces():
    ok = True

    def test_serial_port(glob, name):
        try:
            with open_serial_port(glob):
                info('%s port is OK', name)
                return True
        except Exception:
            error('%s port is not working', name)
            return False

    info('Checking interfaces...')
    ok = test_serial_port(DEBUGGER_PORT_GDB_GLOB, 'GDB') and ok
    ok = test_serial_port(DEBUGGER_PORT_CLI_GLOB, 'CLI') and ok
    try:
        init_can_iface()
        info('CAN interface is OK')
    except Exception:
        logging.debug('CAN check error', exc_info=True)
        error('CAN interface is not working')
        ok = False

    if not ok:
        fatal('Required interfaces are not available. Please check your hardware configuration. '
              'If this application is running on a virtual machine, make sure that hardware '
              'sharing is configured correctly.')

check_interfaces()

licensing_api = make_api_context_with_user_provided_credentials()

with CLIWaitCursor():
    print('Please wait...')
    if args.firmware:
        firmware_data = download(args.firmware)
    else:
        firmware_data = download_newest(DEFAULT_FIRMWARE_GLOB)
    assert 30 < (len(firmware_data) / 1024) <= 240, 'Invalid firmware size'


def process_one_device():
    out = input('1. Connect DroneCode Probe to the debug connector.\n'
                '2. Connect CAN to the first CAN1 connector on the device; terminate the other CAN1 connector.\n'
                '4. Connect an appropriate power supply (see the hardware specs for requirements).\n'
                '   Make sure the motor leads are not connected to anything.\n'
                '5. If you want to skip firmware upload, type F.\n'
                '6. Press ENTER.')

    skip_fw_upload = 'f' in out.lower()
    if not skip_fw_upload:
        info('Loading the firmware')
        with CLIWaitCursor():
            load_firmware_via_gdb(firmware_data,
                                  toolchain_prefix=TOOLCHAIN_PREFIX,
                                  load_offset=FLASH_OFFSET,
                                  gdb_port=glob_one(DEBUGGER_PORT_GDB_GLOB),
                                  gdb_monitor_scan_command='swdp_scan')
    else:
        info('Firmware upload skipped, rebooting the board')
        with open_serial_port(DEBUGGER_PORT_CLI_GLOB) as io:
            SerialCLI(io, 0.1).write_line_and_read_output_lines_until_timeout('reboot')

    info('Waiting for the board to boot...')
    wait_for_boot()

    input('Connect a motor WITHOUT ANY LOAD ATTACHED to the ESC, then press ENTER.\n'
          'CAUTION: THE MOTOR WILL SPIN')

    info('Testing UAVCAN interface...')
    test_uavcan()

    info('Connecting via CLI...')
    with open_serial_port(DEBUGGER_PORT_CLI_GLOB) as io:
        cli = SerialCLI(io, 0.1)
        cli.flush_input(0.5)

        try:
            # Using first command to get rid of any garbage lingering in the buffers
            cli.write_line_and_read_output_lines_until_timeout('systime')
        except Exception:
            pass

        unique_id = b64decode(read_zubax_id(cli)['hw_unique_id'])

        # Getting the signature
        info('Requesting signature for unique ID %s', binascii.hexlify(unique_id).decode())
        gensign_response = licensing_api.generate_signature(unique_id, PRODUCT_NAME)
        if gensign_response.new:
            info('New signature has been generated')
        else:
            info('This particular device has been signed earlier, reusing existing signature')
        base64_signature = b64encode(gensign_response.signature).decode()
        logger.info('Generated signature in Base64: %s', base64_signature)

        # Installing the signature; this may fail if the device has been signed earlier - the failure will be ignored
        out = cli.write_line_and_read_output_lines_until_timeout('zubax_id %s', base64_signature)
        logger.debug('Signature installation response (may fail, which is OK): %r', out)

        # Reading the signature back and verifying it
        installed_signature = read_zubax_id(cli)['hw_signature']
        logger.info('Installed signature in Base64: %s', installed_signature)
        enforce(b64decode(installed_signature) == gensign_response.signature,
                'Written signature does not match the generated signature')

        info('Signature has been installed and verified')

run(process_one_device)
