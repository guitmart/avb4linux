#!/bin/bash
# AVB START - VERSION 64 CANAUX OPTIMISÉE

export AVBHOME=/home/martin/.avb
export INTERFACE=$1
export SAMPLERATE=$2

if [[ $EUID -ne 0 ]]; then echo "Lancer avec sudo"; exit 1; fi
if [ -z "$INTERFACE" ]; then echo "Usage: sudo ./start.sh <interface> <rate>"; exit 1; fi

echo "--- 1. Nettoyage complet ---"
pkill -9 avb-user daemon_cl mrpd maap_daemon shaper_daemon 2>/dev/null
rmmod igb_avb 2>/dev/null
rmmod igb 2>/dev/null
sleep 1

echo "--- 2. Chargement du Driver AVB ---"
modprobe ptp
modprobe i2c_algo_bit
insmod $AVBHOME/igb_avb.ko samplerate=$SAMPLERATE tx_size=1024 InterruptThrottleRate=0
sleep 5

echo "--- 3. Configuration Interface & Ring Buffers ---"
ifconfig $INTERFACE up promisc
ethtool --set-eee $INTERFACE eee off 2>/dev/null

echo "--- 4. Configuration des Files d'attente (Queues) et CPU ---"
if [ -d "/sys/class/net/$INTERFACE/queues/" ]; then
    echo ff > /sys/class/net/$INTERFACE/queues/tx-0/xps_cpus 2>/dev/null
    echo ff > /sys/class/net/$INTERFACE/queues/tx-1/xps_cpus 2>/dev/null
    echo 80 > /sys/class/net/$INTERFACE/queues/tx-2/xps_cpus 2>/dev/null
    echo 80 > /sys/class/net/$INTERFACE/queues/tx-3/xps_cpus 2>/dev/null
    echo "Queues XPS configurées (Audio -> CPU 7)."
fi

# Fixation de l'Interruption (IRQ) sur le CPU 7
# On cherche l'IRQ spécifique TxRx-1 pour l'audio
IRQ=$(grep "$INTERFACE-TxRx-1" /proc/interrupts | awk '{print $1}' | tr -d ':')
if [ -n "$IRQ" ]; then
    echo 80 > /proc/irq/$IRQ/smp_affinity
    echo "Interruption $IRQ fixée sur CPU 7."
fi

echo "--- 5. Lancement des Daemons (Mode Slave Syntonisé) ---"
# -S : Syntonisation (Verrouille la fréquence)
# -R 254 : Force le mode Esclave
$AVBHOME/daemon_cl $INTERFACE -S -R 254 &
sleep 2
$AVBHOME/mrpd -mvs -i $INTERFACE &
$AVBHOME/maap_daemon -i $INTERFACE -d /dev/null &
$AVBHOME/shaper_daemon -d &

echo "--- 6. Attente de Synchronisation Stable (10s) ---"
sleep 10

echo "--- 7. Lancement Audio (Priorité Max sur CPU 7) ---"
taskset -c 7 chrt -f 90 $AVBHOME/avb-user $INTERFACE $SAMPLERATE