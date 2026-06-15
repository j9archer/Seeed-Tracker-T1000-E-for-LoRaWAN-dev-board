#ifndef CREW_LORAWAN_PORTS_H
#define CREW_LORAWAN_PORTS_H

/*
 * Crew tag LoRaWAN FPort policy from MDR-018 and MDR-022.
 * FPort 5 is routine traffic and may be sampled by the relay shim.
 * FPort 6 is alert/uncertain/MOB pass-through traffic and must not be sampled.
 * FPort 7 is routine on-charge/spare traffic and may be sampled separately.
 * FPort 8 is low-rate health/event traffic and should be preserved by the relay shim.
 * FPort 10 is reserved for gateway assistance downlinks.
 * FPort 11 is optional expanded RF fingerprint traffic with deployment-defined filtering.
 */
#define CREW_ROUTINE_APP_PORT       5
#define CREW_ALERT_APP_PORT         6
#define CREW_CHARGER_APP_PORT       7
#define CREW_HEALTH_EVENT_APP_PORT  8
#define GATEWAY_ASSISTANCE_PORT     10
#define CREW_RF_FINGERPRINT_PORT    11

#endif /* CREW_LORAWAN_PORTS_H */
