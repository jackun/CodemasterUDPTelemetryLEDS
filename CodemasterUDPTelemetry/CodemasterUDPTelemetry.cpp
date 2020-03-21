#ifdef _WIN32

#include "stdafx.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#include "wgetopt.h"
#include "Serial.h"

#else

#include <getopt.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include "LinuxSerial.h"
#endif

#include <algorithm>
#include <iostream>
#include <sstream>
#include <iterator>
#include <cstdint>
#include <vector>
#include <thread>
#include <chrono>


// LED setup, expects 36 on top/bottom, 21 on sides
#define NUM_LEDS 114

// https://forums.codemasters.com/topic/30601-f1-2018-udp-specification/


#pragma pack(push,1)
uint8_t lo = (NUM_LEDS - 1) & 0xFF;
uint8_t hi = ((NUM_LEDS - 1) >> 8) & 0xFF;
uint8_t chk = hi ^ lo ^ 0x55;

struct AdaHeader
{
	uint8_t ada[3] = { 'A', 'd', 'a' };
	uint8_t hi;
	uint8_t lo;
	uint8_t chk;
};

struct MyRGB
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

std::ostream & operator<< (std::ostream &out, MyRGB const &t)
{
	out << t.r << t.g << t.b;
	return out;
}

typedef void(*SetLeds)(char * buf, Serial& serial, std::vector<MyRGB>& vec);

void WriteLeds(Serial &serial, std::vector<MyRGB>& vec)
{
	std::stringstream writeBuffer;
	writeBuffer << "Ada";
	writeBuffer << hi << lo << chk;

	std::copy(vec.begin(), vec.end(), std::ostream_iterator<MyRGB>(writeBuffer));

	std::string str = writeBuffer.str();
	//std::cerr << "Sending to serial..." << std::flush;
	serial.WriteData(str.c_str(), str.size());
	//std::cerr << "OK" << std::endl;
}

struct F1_2018 {

	enum PACKETID {
		PID_MOTION = 0,
		PID_SESSION,
		PID_LAP_DATA,
		PID_EVENT,
		PID_PARTICIPANTS,
		PID_CAR_SETUPS,
		PID_CAR_TELEMETRY,
		PID_CAR_STATUS
	};

	struct PacketHeader
	{
		uint16_t    m_packetFormat;         // 2018
		uint8_t     m_packetVersion;        // Version of this packet type, all start from 1
		uint8_t     m_packetId;             // Identifier for the packet type, see below
		uint64_t    m_sessionUID;           // Unique identifier for the session
		float       m_sessionTime;          // Session timestamp
		uint32_t    m_frameIdentifier;      // Identifier for the frame the data was retrieved on
		uint8_t     m_playerCarIndex;       // Index of player's car in the array
	};

	struct PacketHeader2019
	{
		uint16_t    m_packetFormat;         // 2019
		uint8_t     m_gameMajorVersion;     // Game major version - "X.00"
		uint8_t     m_gameMinorVersion;     // Game minor version - "1.XX"
		uint8_t     m_packetVersion;        // Version of this packet type, all start from 1
		uint8_t     m_packetId;             // Identifier for the packet type, see below
		uint64_t    m_sessionUID;           // Unique identifier for the session
		float       m_sessionTime;          // Session timestamp
		uint32_t    m_frameIdentifier;      // Identifier for the frame the data was retrieved on
		uint8_t     m_playerCarIndex;       // Index of player's car in the array
	};

	struct CarMotionData
	{
		float         m_worldPositionX;           // World space X position
		float         m_worldPositionY;           // World space Y position
		float         m_worldPositionZ;           // World space Z position
		float         m_worldVelocityX;           // Velocity in world space X
		float         m_worldVelocityY;           // Velocity in world space Y
		float         m_worldVelocityZ;           // Velocity in world space Z
		int16_t       m_worldForwardDirX;         // World space forward X direction (normalised)
		int16_t       m_worldForwardDirY;         // World space forward Y direction (normalised)
		int16_t       m_worldForwardDirZ;         // World space forward Z direction (normalised)
		int16_t       m_worldRightDirX;           // World space right X direction (normalised)
		int16_t       m_worldRightDirY;           // World space right Y direction (normalised)
		int16_t       m_worldRightDirZ;           // World space right Z direction (normalised)
		float         m_gForceLateral;            // Lateral G-Force component
		float         m_gForceLongitudinal;       // Longitudinal G-Force component
		float         m_gForceVertical;           // Vertical G-Force component
		float         m_yaw;                      // Yaw angle in radians
		float         m_pitch;                    // Pitch angle in radians
		float         m_roll;                     // Roll angle in radians
	};

	struct PacketMotionData
	{
		PacketHeader    m_header;               // Header

		CarMotionData   m_carMotionData[20];    // Data for all cars on track

												// Extra player car ONLY data
		float         m_suspensionPosition[4];       // Note: All wheel arrays have the following order:
		float         m_suspensionVelocity[4];       // RL, RR, FL, FR
		float         m_suspensionAcceleration[4];   // RL, RR, FL, FR
		float         m_wheelSpeed[4];               // Speed of each wheel
		float         m_wheelSlip[4];                // Slip ratio for each wheel
		float         m_localVelocityX;              // Velocity in local space
		float         m_localVelocityY;              // Velocity in local space
		float         m_localVelocityZ;              // Velocity in local space
		float         m_angularVelocityX;            // Angular velocity x-component
		float         m_angularVelocityY;            // Angular velocity y-component
		float         m_angularVelocityZ;            // Angular velocity z-component
		float         m_angularAccelerationX;        // Angular velocity x-component
		float         m_angularAccelerationY;        // Angular velocity y-component
		float         m_angularAccelerationZ;        // Angular velocity z-component
		float         m_frontWheelsAngle;            // Current front wheels angle in radians
	};

	struct MarshalZone
	{
		float  m_zoneStart;   // Fraction (0..1) of way through the lap the marshal zone starts
		int8_t m_zoneFlag;    // -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow, 4 = red
	};

	struct PacketSessionData
	{
		PacketHeader    m_header;               	// Header

		uint8_t         m_weather;              	// Weather - 0 = clear, 1 = light cloud, 2 = overcast
													// 3 = light rain, 4 = heavy rain, 5 = storm
		int8_t   m_trackTemperature;    	// Track temp. in degrees celsius
		int8_t   m_airTemperature;      	// Air temp. in degrees celsius
		uint8_t         m_totalLaps;           	// Total number of laps in this race
		uint16_t        m_trackLength;           	// Track length in metres
		uint8_t         m_sessionType;         	// 0 = unknown, 1 = P1, 2 = P2, 3 = P3, 4 = Short P
												// 5 = Q1, 6 = Q2, 7 = Q3, 8 = Short Q, 9 = OSQ
												// 10 = R, 11 = R2, 12 = Time Trial
		int8_t          m_trackId;         		// -1 for unknown, 0-21 for tracks, see appendix
		uint8_t         m_era;                  	// Era, 0 = modern, 1 = classic
		uint16_t        m_sessionTimeLeft;    	// Time left in session in seconds
		uint16_t        m_sessionDuration;     	// Session duration in seconds
		uint8_t         m_pitSpeedLimit;      	// Pit speed limit in kilometres per hour
		uint8_t         m_gamePaused;               // Whether the game is paused
		uint8_t         m_isSpectating;        	// Whether the player is spectating
		uint8_t         m_spectatorCarIndex;  	// Index of the car being spectated
		uint8_t         m_sliProNativeSupport;	// SLI Pro support, 0 = inactive, 1 = active
		uint8_t         m_numMarshalZones;         	// Number of marshal zones to follow
		MarshalZone     m_marshalZones[21];         // List of marshal zones – max 21
		uint8_t         m_safetyCarStatus;          // 0 = no safety car, 1 = full safety car
													// 2 = virtual safety car
		uint8_t        m_networkGame;              // 0 = offline, 1 = online
	};

	struct PacketSessionData2019
	{
		PacketHeader2019  m_header;              // Header

		uint8_t           m_weather;             // Weather - 0 = clear, 1 = light cloud, 2 = overcast
		                                         // 3 = light rain, 4 = heavy rain, 5 = storm
		int8_t            m_trackTemperature;    // Track temp. in degrees celsius
		int8_t            m_airTemperature;      // Air temp. in degrees celsius
		uint8_t           m_totalLaps;           // Total number of laps in this race
		uint16_t          m_trackLength;         // Track length in metres
		uint8_t           m_sessionType;         // 0 = unknown, 1 = P1, 2 = P2, 3 = P3, 4 = Short P
		                                         // 5 = Q1, 6 = Q2, 7 = Q3, 8 = Short Q, 9 = OSQ
		                                         // 10 = R, 11 = R2, 12 = Time Trial
		int8_t            m_trackId;             // -1 for unknown, 0-21 for tracks, see appendix
		uint8_t           m_formula;             // Formula, 0 = F1 Modern, 1 = F1 Classic, 2 = F2, 3 = F1 Generic
		uint16_t          m_sessionTimeLeft;     // Time left in session in seconds
		uint16_t          m_sessionDuration;     // Session duration in seconds
		uint8_t           m_pitSpeedLimit;       // Pit speed limit in kilometres per hour
		uint8_t           m_gamePaused;          // Whether the game is paused
		uint8_t           m_isSpectating;        // Whether the player is spectating
		uint8_t           m_spectatorCarIndex;   // Index of the car being spectated
		uint8_t           m_sliProNativeSupport; // SLI Pro support, 0 = inactive, 1 = active
		uint8_t           m_numMarshalZones;     // Number of marshal zones to follow
		MarshalZone       m_marshalZones[21];    // List of marshal zones – max 21
		uint8_t           m_safetyCarStatus;     // 0 = no safety car, 1 = full safety car, 2 = virtual safety car
		uint8_t           m_networkGame;         // 0 = offline, 1 = online
	};

	// CAR TELEMETRY PACKET
	struct CarTelemetryData2019;
	struct CarTelemetryData2018
	{
		uint16_t    m_speed;                      // Speed of car in kilometres per hour
		uint8_t     m_throttle;                   // Amount of throttle applied (0 to 100)
		int8_t      m_steer;                      // Steering (-100 (full lock left) to 100 (full lock right))
		uint8_t     m_brake;                      // Amount of brake applied (0 to 100)
		uint8_t     m_clutch;                     // Amount of clutch applied (0 to 100)
		int8_t      m_gear;                       // Gear selected (1-8, N=0, R=-1)
		uint16_t    m_engineRPM;                  // Engine RPM
		uint8_t     m_drs;                        // 0 = off, 1 = on
		uint8_t     m_revLightsPercent;           // Rev lights indicator (percentage)
		uint16_t    m_brakesTemperature[4];       // Brakes temperature (celsius)
		uint16_t    m_tyresSurfaceTemperature[4]; // Tyres surface temperature (celsius)
		uint16_t    m_tyresInnerTemperature[4];   // Tyres inner temperature (celsius)
		uint16_t    m_engineTemperature;          // Engine temperature (celsius)
		float       m_tyresPressure[4];           // Tyres pressure (PSI)

		CarTelemetryData2018() {}
		CarTelemetryData2018(const CarTelemetryData2019& o)
		{
			m_speed = o.m_speed;
			m_throttle = static_cast<uint8_t>(o.m_throttle * 100);
			m_steer = static_cast<int8_t>(o.m_steer * 100);
			m_brake = static_cast<uint8_t>(o.m_brake * 100);
			m_clutch = o.m_clutch;
			m_gear = o.m_gear;
			m_engineRPM = o.m_engineRPM;
			m_drs = o.m_drs;
			m_revLightsPercent = o.m_revLightsPercent;
			memcpy(m_brakesTemperature, o.m_brakesTemperature, sizeof(m_brakesTemperature));
			memcpy(m_tyresSurfaceTemperature, o.m_tyresSurfaceTemperature, sizeof(m_tyresSurfaceTemperature));
			memcpy(m_tyresInnerTemperature, o.m_tyresInnerTemperature, sizeof(m_tyresInnerTemperature));
			m_engineTemperature = o.m_engineTemperature;
			memcpy(m_tyresPressure, o.m_tyresPressure, sizeof(m_tyresPressure));
		}
	};

	struct CarTelemetryData2019
	{
		uint16_t    m_speed;                    // Speed of car in kilometres per hour
		float       m_throttle;                 // Amount of throttle applied (0.0 to 1.0)
		float       m_steer;                    // Steering (-1.0 (full lock left) to 1.0 (full lock right))
		float       m_brake;                    // Amount of brake applied (0.0 to 1.0)
		uint8_t     m_clutch;                   // Amount of clutch applied (0 to 100)
		int8_t      m_gear;                     // Gear selected (1-8, N=0, R=-1)
		uint16_t    m_engineRPM;                // Engine RPM
		uint8_t     m_drs;                      // 0 = off, 1 = on
		uint8_t     m_revLightsPercent;         // Rev lights indicator (percentage)
		uint16_t    m_brakesTemperature[4];     // Brakes temperature (celsius)
		uint16_t    m_tyresSurfaceTemperature[4]; // Tyres surface temperature (celsius)
		uint16_t    m_tyresInnerTemperature[4]; // Tyres inner temperature (celsius)
		uint16_t    m_engineTemperature;        // Engine temperature (celsius)
		float       m_tyresPressure[4];         // Tyres pressure (PSI)
		uint8_t     m_surfaceType[4];           // Driving surface, see appendices
	};

	struct PacketCarTelemetryData2018
	{
		PacketHeader         m_header;                // Header
		CarTelemetryData2018 m_carTelemetryData[20];
		uint32_t             m_buttonStatus;         // Bit flags specifying which buttons are being
													// pressed currently - see appendices
	};

	struct PacketCarTelemetryData2019
	{
		PacketHeader2019     m_header;                // Header
		CarTelemetryData2019 m_carTelemetryData[20];
		uint32_t             m_buttonStatus;         // Bit flags specifying which buttons are being
													// pressed currently - see appendices
	};

	struct PacketCarTelemetryData
	{
		union {
			PacketCarTelemetryData2018 t_18;
			PacketCarTelemetryData2019 t_19;
		} u;
	};
	// CAR STATUS PACKET

	struct CarStatusData
	{
		uint8_t       m_tractionControl;          // 0 (off) - 2 (high)
		uint8_t       m_antiLockBrakes;           // 0 (off) - 1 (on)
		uint8_t       m_fuelMix;                  // Fuel mix - 0 = lean, 1 = standard, 2 = rich, 3 = max
		uint8_t       m_frontBrakeBias;           // Front brake bias (percentage)
		uint8_t       m_pitLimiterStatus;         // Pit limiter status - 0 = off, 1 = on
		float       m_fuelInTank;               // Current fuel mass
		float       m_fuelCapacity;             // Fuel capacity
		uint16_t      m_maxRPM;                   // Cars max RPM, point of rev limiter
		uint16_t      m_idleRPM;                  // Cars idle RPM
		uint8_t       m_maxGears;                 // Maximum number of gears
		uint8_t       m_drsAllowed;               // 0 = not allowed, 1 = allowed, -1 = unknown
		uint8_t       m_tyresWear[4];             // Tyre wear percentage
		uint8_t       m_tyreCompound;             // Modern - 0 = hyper soft, 1 = ultra soft
												// 2 = super soft, 3 = soft, 4 = medium, 5 = hard
												// 6 = super hard, 7 = inter, 8 = wet
												// Classic - 0-6 = dry, 7-8 = wet
		uint8_t       m_tyresDamage[4];           // Tyre damage (percentage)
		uint8_t       m_frontLeftWingDamage;      // Front left wing damage (percentage)
		uint8_t       m_frontRightWingDamage;     // Front right wing damage (percentage)
		uint8_t       m_rearWingDamage;           // Rear wing damage (percentage)
		uint8_t       m_engineDamage;             // Engine damage (percentage)
		uint8_t       m_gearBoxDamage;            // Gear box damage (percentage)
		uint8_t       m_exhaustDamage;            // Exhaust damage (percentage)
		int8_t        m_vehicleFiaFlags;          // -1 = invalid/unknown, 0 = none, 1 = green
												// 2 = blue, 3 = yellow, 4 = red
		float       m_ersStoreEnergy;           // ERS energy store in Joules
		uint8_t       m_ersDeployMode;            // ERS deployment mode, 0 = none, 1 = low, 2 = medium
												// 3 = high, 4 = overtake, 5 = hotlap
		float       m_ersHarvestedThisLapMGUK;  // ERS energy harvested this lap by MGU-K
		float       m_ersHarvestedThisLapMGUH;  // ERS energy harvested this lap by MGU-H
		float       m_ersDeployedThisLap;       // ERS energy deployed this lap
	};

	struct CarStatusData2019
	{
		uint8_t       m_tractionControl;          // 0 (off) - 2 (high)
		uint8_t       m_antiLockBrakes;           // 0 (off) - 1 (on)
		uint8_t       m_fuelMix;                  // Fuel mix - 0 = lean, 1 = standard, 2 = rich, 3 = max
		uint8_t       m_frontBrakeBias;           // Front brake bias (percentage)
		uint8_t       m_pitLimiterStatus;         // Pit limiter status - 0 = off, 1 = on
		float       m_fuelInTank;               // Current fuel mass
		float       m_fuelCapacity;             // Fuel capacity
		float       m_fuelRemainingLaps;        // Fuel remaining in terms of laps (value on MFD)
		uint16_t      m_maxRPM;                   // Cars max RPM, point of rev limiter
		uint16_t      m_idleRPM;                  // Cars idle RPM
		uint8_t       m_maxGears;                 // Maximum number of gears
		uint8_t       m_drsAllowed;               // 0 = not allowed, 1 = allowed, -1 = unknown
		uint8_t       m_tyresWear[4];             // Tyre wear percentage
		uint8_t       m_actualTyreCompound;	   // F1 Modern - 16 = C5, 17 = C4, 18 = C3, 19 = C2, 20 = C1
						   // 7 = inter, 8 = wet
						   // F1 Classic - 9 = dry, 10 = wet
						   // F2 – 11 = super soft, 12 = soft, 13 = medium, 14 = hard
						   // 15 = wet
		uint8_t       m_tyreVisualCompound;       // F1 visual (can be different from actual compound)
		   // 16 = soft, 17 = medium, 18 = hard, 7 = inter, 8 = wet
		   // F1 Classic – same as above
		   // F2 – same as above
		uint8_t       m_tyresDamage[4];           // Tyre damage (percentage)
		uint8_t       m_frontLeftWingDamage;      // Front left wing damage (percentage)
		uint8_t       m_frontRightWingDamage;     // Front right wing damage (percentage)
		uint8_t       m_rearWingDamage;           // Rear wing damage (percentage)
		uint8_t       m_engineDamage;             // Engine damage (percentage)
		uint8_t       m_gearBoxDamage;            // Gear box damage (percentage)
		int8_t        m_vehicleFiaFlags;	   // -1 = invalid/unknown, 0 = none, 1 = green
												// 2 = blue, 3 = yellow, 4 = red
		float       m_ersStoreEnergy;           // ERS energy store in Joules
		uint8_t       m_ersDeployMode;            // ERS deployment mode, 0 = none, 1 = low, 2 = medium
						   // 3 = high, 4 = overtake, 5 = hotlap
		float       m_ersHarvestedThisLapMGUK;  // ERS energy harvested this lap by MGU-K
		float       m_ersHarvestedThisLapMGUH;  // ERS energy harvested this lap by MGU-H
		float       m_ersDeployedThisLap;       // ERS energy deployed this lap
	};

	struct PacketCarStatusData
	{
		PacketHeader        m_header;            // Header

		CarStatusData       m_carStatusData[20];
	};

	struct PacketCarStatusData2019
	{
		PacketHeader2019    m_header;            // Header

		CarStatusData2019   m_carStatusData[20];
	};

	static void SetLeds(char * buf, Serial& serial, std::vector<MyRGB>& vec)
	{
		static uint16_t maxRPM = 0;
		static uint8_t lastFormula = 0;
		PacketHeader *h = reinterpret_cast<PacketHeader *>(buf);
		//std::cout << "received " << rlen << std::endl;

		int packetId = (h->m_packetFormat == 2019) ? reinterpret_cast<PacketHeader2019 *>(h)->m_packetId : h->m_packetId;
		std::cout << "Header " << h->m_packetFormat << " version " << (unsigned int)h->m_packetVersion << " id " << packetId << std::endl;

		switch (packetId)
		{
		case PID_SESSION:
		{
			if (h->m_packetFormat == 2018)
			{
				PacketSessionData *p = reinterpret_cast<PacketSessionData *>(buf);
				lastFormula = p->m_era;
			}
			else if (h->m_packetFormat == 2019)
			{
				PacketSessionData2019 *p = reinterpret_cast<PacketSessionData2019 *>(buf);
				lastFormula = p->m_formula;
			}
		}
		break;
		case PID_CAR_STATUS:
		{
			if (h->m_packetFormat == 2018)
			{
				PacketCarStatusData *p = reinterpret_cast<PacketCarStatusData *>(buf);
				auto s = p->m_carStatusData[p->m_header.m_playerCarIndex];
				maxRPM = s.m_maxRPM;
			}
			else if (h->m_packetFormat == 2019)
			{
				PacketCarStatusData2019 *p = reinterpret_cast<PacketCarStatusData2019 *>(buf);
				auto s = p->m_carStatusData[p->m_header.m_playerCarIndex];
				maxRPM = s.m_maxRPM;
			}
		}
		break;

		case PID_CAR_TELEMETRY:
		{
			PacketCarTelemetryData *p = reinterpret_cast<PacketCarTelemetryData *>(buf);
			CarTelemetryData2018 s {};

			if (h->m_packetFormat == 2018)
				s = p->u.t_18.m_carTelemetryData[p->u.t_18.m_header.m_playerCarIndex];
			else if (h->m_packetFormat == 2019)
				s = CarTelemetryData2018(p->u.t_19.m_carTelemetryData[p->u.t_19.m_header.m_playerCarIndex]);
			else
				return;

			std::cout << "RPM " << s.m_engineRPM << " / " << maxRPM << "(" << (int)s.m_revLightsPercent << ")" << std::endl;

			if (maxRPM && serial.IsConnected()) {
				//int percent = s.m_revLightsPercent;
				int promill = s.m_engineRPM * 1000 / maxRPM;
				std::cout << "\t percent: " << promill << std::endl;
				promill = std::min(promill, 1000);

				memset(vec.data(), 0, vec.size() * sizeof(MyRGB));
				MyRGB led;

				if (promill <= 600)
				{
					led.r = 0;
					led.g = 25 + ((230 * promill) / 600);
					led.b = 0;
				}
				else if (promill <= 750)
				{
					led.r = (255);
					led.g = (255);
					led.b = 0;
				}
				else if (promill <= 875 - (s.m_gear * 4))
				{
					led.r = (255);
					led.g = 0;
					led.b = 0;
				}
				else
				{
					led.r = (128);
					led.g = 0;
					led.b = (255);
				}

				// sides
				for (int i = 0; i < (21 * promill) / 1000; i++)
				{
					vec[36 + i] = led; // left
					vec[36 + 21 + 36 + 21 - i - 1] = led; //right
				}

				// top/bottom
				for (int i = 0; i < (18 * promill) / 1000; i++)
				{
					vec[36 - i - 1] = led; // from left
					vec[i] = led; //from right

					vec[36 + 21 + 36 - i - 1] = led; // from left
					vec[36 + 21 + i] = led; //from right
				}

				WriteLeds(serial, vec);
			}
		}
		break;
		default:
			break;
		}
	}
};


struct DIRT
{
	// needs extradata = 3 in config xml
	struct Telemetry
	{
		//float data[64];
		float m_time;	// Time
		float m_lapTime;	// Time of Current Lap
		float m_lapDistance;	// Distance Driven on Current Lap
		float m_totalDistance;	// Distance Driven Overall
		float m_x; // World space position	Position X
		float m_y; // World space position	Position Y
		float m_z; // World space position	Position Z
		float m_speed;	// Velocity(Speed)[m / s]
		float m_xv; // Velocity in world space	Velocity X
		float m_yv; // Velocity in world space	Velocity Y
		float m_zv; // Velocity in world space	Velocity Z
		float m_xr; // World space right direction	Roll Vector X
		float m_yr; // World space right direction	Roll Vector Y
		float m_zr; // World space right direction	Roll Vector Z
		float m_xd; // World space forward direction	Pitch Vector X
		float m_yd; // World space forward direction	Pitch Vector Y
		float m_zd; // World space forward direction	Pitch Vector Z
		float m_susp_pos_bl;	// Position of Suspension Rear Left
		float m_susp_pos_br;	// Position of Suspension Rear Right
		float m_susp_pos_fl;	// Position of Suspension Front Left
		float m_susp_pos_fr;	// Position of Suspension Front Right
		float m_susp_vel_bl;	// Velocity of Suspension Rear Left
		float m_susp_vel_br;	// Velocity of Suspension Rear Right
		float m_susp_vel_fl;	// Velocity of Suspension Front Left
		float m_susp_vel_fr;	// Velocity of Suspension Front Right
		float m_wheel_speed_bl;	// Velocity of Wheel Rear Left
		float m_wheel_speed_br;	// Velocity of Wheel Rear Right
		float m_wheel_speed_fl;	// Velocity of Wheel Front Left
		float m_wheel_speed_fr;	// Velocity of Wheel Front Right
		float m_throttle;	// Position Throttle
		float m_steer;	// Position Steer
		float m_brake;	// Position Brake
		float m_clutch;	// Position Clutch
		float m_gear;	// Gear[0 = Neutral, 1 = 1, 2 = 2, ..., 10 = Reverse]
		float m_gforce_lat;	// G - Force Lateral
		float m_gforce_lon;	// G - Force Longitudinal
		float m_lap;	// Current Lap
		float m_engineRate;	// Speed of Engine[rpm / 10]
		float m_sli_pro_native_support; // SLI Pro support	?
		float m_car_position; // car race position	?
		float m_kers_level; // kers energy left	?
		float m_kers_max_level; // kers maximum energy	?
		float m_drs; // 0 = off, 1 = on	?
		float m_traction_control; // 0 (off) - 2 (high)	?
		float m_anti_lock_brakes; // 0 (off) - 1 (on)	?
		float m_fuel_in_tank; // current fuel mass	?
		float m_fuel_capacity; // fuel capacity	?
		float m_in_pits; // 0 = none, 1 = pitting, 2 = in pit area	?
		float m_sector; // 0 = sector1, 1 = sector2; 2 = sector3	?
		float m_sector1_time; // time of sector1 (or 0)	?
		float m_sector2_time; // time of sector2 (or 0)	?
		float m_brakes_temp[4]; // brakes temperature (centigrade)	Temperature Brake Rear Left ?
		float m_wheels_pressure[4]; // wheels pressure PSI	Temperature Brake Rear Right ?
		float m_team_info; // team ID	Temperature Brake Front Left ?
		float m_total_laps; // total number of laps in this race	Temperature Brake Front Right ?
		float m_track_size; // track size meters	?
		float m_last_lap_time; // last lap time	?
		float m_max_rpm; // cars max RPM, at which point the rev limiter will kick in	?
		float m_idle_rpm; // cars idle RPM	?
		float m_max_gears; // maximum number of gears	?
		float m_sessionType; // 0 = unknown, 1 = practice, 2 = qualifying, 3 = race	Number of Laps in Total ?
		float m_drsAllowed; // 0 = not allowed, 1 = allowed, -1 = invalid / unknown	Length of Track in Total
		float m_track_number; // -1 for unknown, 0-21 for tracks	?
		float m_vehicleFIAFlags; // -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow, 4 = red	Maximum rpm / 10
	};

	static void SetLeds(char * buf, Serial& serial, std::vector<MyRGB>& vec)
	{
		Telemetry *t = reinterpret_cast<Telemetry *>(buf);
		std::cout << "gear " << t->m_gear << " RPM " << t->m_engineRate * 10 << " max RPM " << t->m_max_rpm * 10 << std::endl;

		if (t->m_max_rpm > 0.f && serial.IsConnected()) {
			//int percent = s.m_revLightsPercent;
			int percent = std::min(100, int(100 * t->m_engineRate / t->m_max_rpm));
			std::cout << "\t percent: " << percent << std::endl;

			memset(vec.data(), 0, vec.size() * sizeof(MyRGB));
			MyRGB led;

			if (percent <= 60)
			{
				led.r = 0;
				led.g = 25 + ((230 * percent) / 60);
				led.b = 0;
			}
			else if (percent <= 85)
			{
				led.r = 255 * (percent - 60) / 25;
				led.g = (255);
				led.b = 0;
			}
			else
			{
				led.r = 255;
				led.g = 255 - 255 * (percent - 85) / 15;
				led.b = 0;
			}

			// sides
			for (int i = 0; i < (21 * percent) / 100; i++)
			{
				vec[36 + i] = led; // left
				vec[36 + 21 + 36 + 21 - i - 1] = led; //right
			}

			// top/bottom
			for (int i = 0; i < (18 * percent) / 100; i++)
			{
				vec[36 - i - 1] = led; // from left
				vec[i] = led; //from right

				vec[36 + 21 + 36 - i - 1] = led; // from left
				vec[36 + 21 + i] = led; //from right
			}

			WriteLeds(serial, vec);
		}
	}
};
#pragma pack(pop)

void AdalightWakeUp(char * buf, Serial& serial, std::vector<MyRGB>& vec)
{
	//memset(vec.data(), 0, sizeof(MyRGB) * vec.size());
	for (auto& rgb : vec) {
		rgb.r = rgb.b = 64;
	}
	WriteLeds(serial, vec);
}

#define SERVER "127.0.0.1"
#define BUFLEN 1500
#define PORT 20777

#ifdef _WIN32
int running = 1;
SOCKET s;
WSADATA wsa;
#define socklen_t int

BOOL WINAPI consoleHandler(DWORD signal) {

	switch (signal) {
	case CTRL_CLOSE_EVENT:
	case CTRL_BREAK_EVENT:
	case CTRL_C_EVENT:
#ifndef NDEBUG
		printf("Ctrl-C handled\n");
		Beep(750, 300);
#endif
		running = 0;
		closesocket(s);
		WSACleanup();
	}

	return TRUE;
}
#else
int s;
#define SOCKADDR sockaddr

#include <signal.h>
volatile sig_atomic_t running = 1;
void sighandler(int sig) {
	if (sig == SIGINT || sig == SIGTERM) {
		running = 0;
		printf("Ctrl-C handled\n");
		shutdown(s, 0);
	}
}
#endif

enum GAME_TYPE
{
	GT_NONE,
	GT_F1_2018,
	GT_DIRT,
	GT_DIRT2
};

int main(int argc, char* const * argv)
{
	struct sockaddr_in si_other;
	socklen_t slen = sizeof(si_other), rlen = 0;
	char buf[BUFLEN];
	//char serialbuf[8];
	uint16_t maxRPM = 0;
	std::vector<MyRGB> rgb4Vec(NUM_LEDS);
	GAME_TYPE game = GT_NONE;
	SetLeds setLeds = nullptr;
	const char *comPort = "COM6";
	const char *server = SERVER;

	int c;
	while ((c = getopt(argc, argv, "hs:p:g:")) != -1)
		switch (c)
		{
		case 'p':
			comPort = optarg;
			break;
		case 's':
			server = optarg;
			break;
		case 'g':
			//aflag = 1;
			if (!strncmp(optarg, "f1_2018", 8)) {
				game = GT_F1_2018;
				setLeds = F1_2018::SetLeds;
			}
			else if (!strncmp(optarg, "dirt_rally2", 6)) {
				game = GT_DIRT2;
				setLeds = DIRT::SetLeds;
			}
			else if (!strncmp(optarg, "dirt_rally", 5)) {
				game = GT_DIRT;
				setLeds = DIRT::SetLeds;
			}
			else if (!strncmp(optarg, "none", 4)) {
				game = GT_NONE;
				setLeds = AdalightWakeUp;
			}
			else
			{
				fprintf(stderr, "Unknown game type '%s'.\n", optarg);
				return 1;
			}
			break;
		case 'h':
			std::cout << "Usage:\n"
				"\t[-s <ip>]\thost IP, defaults to localhost\n"
				"\t[-p <COM port>]\tCOM port, defaults to COM4\n"
				"\t-g <game>\tSet game UDP type\n"
				"\t-h\t\tThis message\n"
				"Games:\n\tf1_2018\n\tdirt_rally\n\tdirt_rally2" << std::endl;
			return 0;
		case '?':
			if (optopt == 'g')
				fprintf(stderr, "Option -%c requires an argument.\n", optopt);
			else if (isprint(optopt))
				fprintf(stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf(stderr,
					"Unknown option character `\\x%x'.\n",
					optopt);
			return 1;
		default:
			abort();
		}

#ifdef _WIN32
	if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
		printf("\nERROR: Could not set control handler\n");
		return 1;
	}
#else
	signal(SIGINT, sighandler);
	signal(SIGTERM, sighandler);
#endif

	Serial serial(comPort);
	/*serial.ReadData(serialbuf, sizeof(serialbuf));
	if (serialbuf[0] == 'A' && serialbuf[1] == 'd' && serialbuf[2] == 'a' && serialbuf[3] == '\n')
	{

	}*/
#ifdef _WIN32
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		return (EXIT_FAILURE);
	}
	printf("OK.\n");
#endif

	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
	{
#ifdef _WIN32
		printf("socket() failed with error code : %d", WSAGetLastError());
#endif
		return (EXIT_FAILURE);
	}
	
	//setup address structure
	memset((char *)&si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	inet_pton(AF_INET, server, &(si_other.sin_addr));

	std::cout << "Binding socket to " << server << "..." << std::flush;
	//int result = connect(s, (SOCKADDR*)&si_other, sizeof(si_other));
	int result = ::bind(s, (SOCKADDR*)&si_other, sizeof(si_other));
	if (result < 0) {
#ifdef _WIN32
		std::cout << "Socket bind failed. Error: " << WSAGetLastError() << std::endl;
#endif
		return (EXIT_FAILURE);
	}

	std::cout << "OK" << std::endl;

	AdalightWakeUp(nullptr, serial, rgb4Vec);

	// expects console handler to break the loop and cleanup WSA afterwards
	while (running)
	{
		memset(buf, 0, BUFLEN);
		//try to receive some data, this is a blocking call
		if (game != GT_NONE && (rlen = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) < 0 && running)
		{
#ifdef _WIN32
			printf("recvfrom() failed with error code : %d\n", WSAGetLastError());
#endif
			return (EXIT_FAILURE);
		}
		/*else {
			rlen = 0;
			static bool firstTime = true;
			static auto time = std::chrono::steady_clock::now();
			auto dur = std::chrono::steady_clock::now() - time;
			if (firstTime || dur >= std::chrono::minutes(5))
			{
				rlen = 1;
				time = std::chrono::steady_clock::now();
				std::cout << "wakey-wakey!" << std::endl;
			} else {
				std::this_thread::sleep_for (std::chrono::milliseconds(500));
			}
			firstTime = false;
		}*/

		std::cout << "rlen " << rlen << std::endl;
		if (rlen > 0 && setLeds) {
			setLeds(buf, serial, rgb4Vec);
		}
	}

    return 0;
}

