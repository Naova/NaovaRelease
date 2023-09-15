/**
 * @file NaovaSPLStandardMessage.h
 *
 * The file integrate the SPLStandardMessage into the Naova system.
 * Ref : BHULKsStandardMessage, BSPLStandardMessage
 * @author <A href="mailto:paul-joseph.beltran.1@ens.etsmtl.ca">Paul Joseph Beltran/A>
 */

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"

struct NaovaSPLStandardMessage : public RoboCup::SPLStandardMessage, public Streamable
{
    static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

    NaovaSPLStandardMessage() : RoboCup::SPLStandardMessage(), Streamable()
    {
        numOfDataBytes = 0;
    }

    size_t sizeofNaovaSPLMessage() const;

    // Method to convert this struct for communication usage
    // @param data point to dataspace,
    //        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBSPLMessage()
    // -asserts: writing sizeOfBHMessage() bytes
    void write(void* data) const;

    // Method to reads the message from data.
    // @param data the message
    // @return the header and the versions are convertible
    bool read(const void* data);

protected:
    void serialize(In* in, Out* out);

private:
    using RoboCup::SPLStandardMessage::data; //hide because it should not be used
};