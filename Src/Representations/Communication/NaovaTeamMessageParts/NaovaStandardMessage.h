/**
 * @file NaovaStandardMessage.h * 
 * @author Paul Joseph Beltran
 * @brief clean data and refactor of BHULKsStandardMessage and BHumanStandardMessage
 * 
 */

#pragma once
#include <stdint.h>
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Role.h"

namespace RoboCup
{
    #include <RoboCupGameControlData.h>
}
namespace Naova
{
#define NAOVA_STANDARD_MESSAGE_STRUCT_HEADER "NAO"
#define NAOVA_STANDARD_MESSAGE_STRUCT_VERSION 2
#define NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS 5
#define NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES 7 //< max send obstacles per messages

    /**
     * Definition of possible confidences based on teh robots hearing capability
     */
    enum class HearingConfidence : uint8_t
    {
        iAmDeaf = (unsigned char)-1,
        heardOnOneEarButThinkingBothAreOk = 33,
        oneEarIsBroken = 66,
        allEarsAreOk = 100
    };
    constexpr HearingConfidence numOfHearingConfidences = HearingConfidence(4);

    /**
     * The type of an obstacle.
     */
    enum class ObstacleType : uint8_t
    {
        goalpost,
        unknown,
        someRobot,
        opponent,
        teammate,
        fallenSomeRobot,
        fallenOpponent,
        fallenTeammate
    };
    constexpr ObstacleType numOfObstacleTypes = ObstacleType(8);

    /**
     * The definintion of an Obstacle as it is shared between B-HULKs-robots.
     */
    struct Obstacle
    {
        // the obstacle center in robot (self) centered coordinates
        // - x goes to front
        // - y goes to left
        float center[2]; // < [short (4mm)]

        uint32_t timestampLastSeen; //< [delta 0..-16384 (64) ] The name says it
        ObstacleType type;          //< [0..numOfObstacleTypes] The name says it

        // returns the size of this struct when it is written
        static int sizeOfObstacle();

        // Method to convert this struct for communication usage
        // @param data point to dataspace,
        // @param timestamp the reference timestamp of the writimg message
        //  -asserts: writing sizeOfObstacle() bytes
        void write(void *&data, uint32_t timestamp) const;

        // Method to reads the Obstacle from data.
        // @param data the Obstacle
        // @param timestamp the reference timestamp of reading message
        void read(const void *&data, uint32_t timestamp);
    };

    /**
     * The definintion of an ntp message we send - in response to a previously received request
     */
    struct BNTPMessage
    {
        uint32_t requestOrigination; //<                        The timestamp of the generation of the request
        uint32_t requestReceipt;     //< [delta 0..-4096]       The timestamp of the receipt of the request
        uint8_t receiver;           //< [#_MAX_NUM_OF_PLAYERS] The robot, to which this message should be sent
        BNTPMessage(){
}; 
    };
    /**
     * A struct that holds the main (useful) data of RoboCupGameControlData.
     * This is used to to spread important data via NaovaStandardMessage for
     *   partly compensation in case of game controller package loss.
     */
    struct OwnTeamInfo
    {
        static_assert(GAMECONTROLLER_STRUCT_VERSION == 15,
                      "Please adjust this struct to the newer game controller struct version");

        OwnTeamInfo();

        // timestamp when RoboCupGameControlData (RoboCup::) was reveived
        uint32_t timestampWhenReceived; // [delta 0..-1.09 minutes (256ms)]

        // values of RoboCup::RoboCupGameControlData
        uint8_t packetNumber;
        uint8_t competitionType;  // < [0..3]
        uint8_t competitionPhase; // < [0..3]
        uint8_t state;            // < [0..7]
        uint8_t setPlay;
        uint8_t firstHalf; // < [0..1]
        uint8_t kickingTeam;
        uint8_t gamePhase; // < [0..3]
        uint8_t dropInTeam;
        uint16_t dropInTime;    // < [0..62 (2)]
        uint16_t secsRemaining; // < [0..1023]
        uint16_t secondaryTime; // < [0..511]

        // values of RoboCup::RoboCupGameControlData::TeamInfo
        uint8_t score;
        bool playersArePenalized[NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS];

        // returns the size of this struct when it is written
        int sizeOfOwnTeamInfo() const;

        // Method to convert this struct for communication usage
        // @param data point to dataspace
        // @param timestamp the reference timestamp of the writimg message
        //  -asserts: writing sizeOfOwnTeamInfo() bytes
        void write(void *&data, uint32_t timestamp) const;

        // Method to reads the OwnTeamInfo from data.
        // @param data the OwnTeamInfo
        // @param timestamp the reference timestamp of reading message
        void read(const void *&data, uint32_t timestamp);
    };
        
    Out& operator<<(Out& stream, const Obstacle& bHSTMObstacle);
    In& operator >> (In& stream, Obstacle& bHSTMObstacle);

    Out& operator<<(Out& stream, const BNTPMessage& ntpMessage);
    In& operator >> (In& stream, BNTPMessage& ntpMessage);

    Out& operator<<(Out& stream, const OwnTeamInfo& ownTeamInfo);
    In& operator >> (In& stream, OwnTeamInfo& ownTeamInfo);

    inline static const char* getName(HearingConfidence e);
    inline static const char* getName(ObstacleType e);
};

struct NaovaStandardMessage : public Streamable
{
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "Please adjust this file to the newer version.");

    int sizeOfNaovaMessage() const;

    // Method to convert this struct for communication usage
    // @param data point to dataspace,
    //        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBHumanMessage()
    // -asserts: writing sizeOfNaovaMessage() bytes
    void write(void *data) const;

    // Method to reads the message from data.
    // @param data the message
    // @return the header and the versions are convertible
    bool read(const void *data);

    NaovaStandardMessage();

    // char header[4];
    // uint8_t version;
    uint8_t magicNumber;

    // uint32_t ballTimeWhenDisappearedSeenPercentage;
    float ballVel[2];
    int16_t ballLastPerceptX;
    int16_t ballLastPerceptY;
    std::array<float, 3> ballCovariance;
    float robotPoseDeviation;
    // std::array<float, 6> robotPoseCovariance;
    uint8_t robotPoseValidity;
    bool isPenalized;
    bool isUpright;
    bool hasGroundContact; 

    uint32_t timestamp;
    // Naova::OwnTeamInfo gameControlData;
    Role::RoleType currentlyPerformingRole;
    // Role::RoleType roleAssignments[NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS];
    // bool keeperIsPlaying;
    // int8_t passTarget;
    // uint32_t timeWhenReachBall;
    // uint32_t timeWhenReachBallStriker;
    uint32_t timestampLastJumped;
    uint32_t ballTimeWhenLastSeen;
    Naova::HearingConfidence confidenceOfLastWhistleDetection;
    uint32_t lastTimeWhistleDetected;
    uint32_t confidenceLevel;
    // std::vector<Naova::Obstacle> obstacles;
    // bool requestsNTPMessage;
    // std::vector<Naova::BNTPMessage> ntpMessages;

protected:
    void serialize(In* in, Out* out);
};