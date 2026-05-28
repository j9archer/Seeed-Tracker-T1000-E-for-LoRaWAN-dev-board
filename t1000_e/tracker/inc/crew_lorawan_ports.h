#ifndef CREW_LORAWAN_PORTS_H
#define CREW_LORAWAN_PORTS_H

/*
 * Crew tag LoRaWAN FPort policy from MDR-018.
 * FPort 5 is routine traffic and may be sampled by the relay shim.
 * FPort 6 is alert/uncertain/MOB pass-through traffic and must not be sampled.
 * FPort 7 is routine on-charge/spare traffic and may be sampled separately.
 * FPort 10 is reserved for gateway assistance downlinks.
 */
#define CREW_ROUTINE_APP_PORT       5
#define CREW_ALERT_APP_PORT         6
#define CREW_CHARGER_APP_PORT       7
#define GATEWAY_ASSISTANCE_PORT     10

#endif /* CREW_LORAWAN_PORTS_H */
