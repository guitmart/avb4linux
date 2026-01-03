#!/bin/bash
# Script de restauration complète (Stop AVB)

export INTERFACE=$1

# Vérification de l'argument
if [ -z "$INTERFACE" ]; then
    echo "Usage: sudo ./stop_avb.sh <interface> (ex: enp3s0)"
    exit 1
fi

echo "--- Arrêt des processus AVB ---"
# On tue tout proprement, le -9 garantit la libération des ressources mémoire
pkill -9 avb-user 2>/dev/null
pkill -9 shaper_daemon 2>/dev/null
pkill -9 maap_daemon 2>/dev/null
pkill -9 mrpd 2>/dev/null
pkill -9 daemon_cl 2>/dev/null

sleep 1

echo "--- Nettoyage Hardware et Modules ---"
ifconfig $INTERFACE down 2>/dev/null

# 1. On décharge les modules pour réinitialiser la puce i210
rmmod igb_avb 2>/dev/null
rmmod igb 2>/dev/null

# 2. Restauration des masques XPS (Queues TX)
# On remet 'f' (ou 'ff' pour 8 coeurs) pour que le trafic soit réparti
if [ -d "/sys/class/net/$INTERFACE/queues/" ]; then
    echo "Restauration des files d'attente réseau..."
    for i in {0..3}; do
        echo "ff" > /sys/class/net/$INTERFACE/queues/tx-$i/xps_cpus 2>/dev/null
    done
fi

# 3. Recharge du driver Intel standard
modprobe igb
sleep 1
ifconfig $INTERFACE up

# 4. Restauration des Interruptions (IRQ)
echo "Restauration de l'équilibrage des interruptions..."
# On récupère l'IRQ de la carte fraîchement rechargée
NEW_IRQ=$(grep "$INTERFACE" /proc/interrupts | awk '{print $1}' | tr -d ':')
for i in $NEW_IRQ; do
    # On remet le masque ff (tous les processeurs autorisés)
    echo "ff" > /proc/irq/$i/smp_affinity 2>/dev/null
done

# 5. Relancer irqbalance
systemctl start irqbalance 2>/dev/null

echo "--- Terminé : Système restauré en mode normal ---"