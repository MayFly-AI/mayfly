#!/bin/env python3
import os, sys, glob
import subprocess

def find_phy_dev(driver_match):
    drivers = glob.glob('/sys/class/ieee80211/*/device/driver/module/drivers/*')
    for driver in drivers:
        if driver_match in driver:
            driver = driver.split('/')
            for d in driver:
                if 'phy' in d:
                    phy_dev = d
                    print('Found physical device "%s" for driver "*%s*"' % (phy_dev, driver_match))
                    return phy_dev
    print('Failed to find physical device for driver "*%s*"' % driver_match)
    return None

def find_device_name(phy):
    net_devices = glob.glob('/sys/class/ieee80211/*/device/net/*')
    for net in net_devices:
        net = net.split("/")
        for w in net:
            if phy == w:
                dev_name = net[-1]
                print('Found adapter: "%s" matching physical device: "%s"' % (dev_name, phy))
                return dev_name
    print('Failed to find wireless device for physical device "%s"' % phy)
    return None

if __name__ == '__main__':
    name = 'rtl88'
    phy_dev = find_phy_dev(name)
    if phy_dev is None:
        exit(1)
    dev_name = find_device_name(phy_dev)
    if dev_name is None:
        exit(1)

    cmds = ['iw reg set BO',
            'ip link set %s down' % dev_name,
            'iw dev %s set monitor otherbss' % dev_name,
            'ip link set %s up' % dev_name,
            'iw dev %s set channel 161 HT40+' % dev_name]

    print('Setting to monitor mode for NIC: "%s"\n' % dev_name)
    for cmd in cmds:
        print(cmd)
    print()

    for cmd in cmds:
        res = subprocess.run(cmd, shell=True)
        if res.returncode != 0:
            uid = os.getuid()
            print('user id: %d - try with root permissions' % uid)
            exit(1)
