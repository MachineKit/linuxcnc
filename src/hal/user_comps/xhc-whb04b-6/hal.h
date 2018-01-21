/*
   Copyright (C) 2017 Raoul Rubien (github.com/rubienr)

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the program; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.
 */

#pragma once

// local includes
#include "pendant-types.h"

// system includes
#include <stdint.h>
#include <ostream>
#include <string>
#include <map>

// 3rd party includes

// local library includes
#include <hal.h>

// forward declarations

namespace XhcWhb04b6 {

// forward declarations
class WhbSoftwareButton;
class WhbKeyCodes;


// ----------------------------------------------------------------------

//! HAL memory pointers. Each pointer represents an i/o hal pin.

class WhbHalMemory
{
public:
    struct In
    {
    public:
        //! to be connected to \ref halui.axis.0.pos-feedback
        hal_float_t* axisXPosition{nullptr};
        //! to be connected to \ref halui.axis.1.pos-feedback
        hal_float_t* axisYPosition{nullptr};
        //! to be connected to \ref halui.axis.2.pos-feedback
        hal_float_t* axisZPosition{nullptr};
        //! to be connected to \ref halui.axis.3.pos-feedback
        hal_float_t* axisAPosition{nullptr};
        //! to be connected to \ref halui.axis.4.pos-feedback
        hal_float_t* axisBPosition{nullptr};
        //! to be connected to \ref halui.axis.5.pos-feedback
        hal_float_t* axisCPosition{nullptr};

        //! to be connected to \ref halui.axis.0.pos-relative
        hal_float_t* axisXPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.1.pos-relative
        hal_float_t* axisYPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.2.pos-relative
        hal_float_t* axisZPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.3.pos-relative
        hal_float_t* axisAPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.4.pos-relative
        hal_float_t* axisBPositionRelative{nullptr};
        //! to be connected to \ref halui.axis.5.pos-relative
        hal_float_t* axisCPositionRelative{nullptr};

        //! to be connected to \ref stepgen.00.maxvel
        hal_float_t* stepgenXMaxVelocity{nullptr};
        //! to be connected to \ref stepgen.01.maxvel
        hal_float_t* stepgenYMaxVelocity{nullptr};
        //! to be connected to \ref stepgen.02.maxvel
        hal_float_t* stepgenZMaxVelocity{nullptr};
        //! to be connected to \ref stepgen.03.maxvel
        hal_float_t* stepgenAMaxVelocity{nullptr};
        //! to be connected to \ref stepgen.04.maxvel
        hal_float_t* stepgenBMaxVelocity{nullptr};
        //! to be connected to \ref stepgen.05.maxvel
        hal_float_t* stepgenCMaxVelocity{nullptr};
        //! to be connected to \ref stepgen.00.position-scale
        hal_float_t* stepgenXPositionScale{nullptr};
        //! to be connected to \ref stepgen.01.position-scale
        hal_float_t* stepgenYPositionScale{nullptr};
        //! to be connected to \ref stepgen.02.position-scale
        hal_float_t* stepgenZPositionScale{nullptr};
        //! to be connected to \ref stepgen.03.position-scale
        hal_float_t* stepgenAPositionScale{nullptr};
        //! to be connected to \ref stepgen.04.position-scale
        hal_float_t* stepgenBPositionScale{nullptr};
        //! to be connected to \ref stepgen.05.position-scale
        hal_float_t* stepgenCPositionScale{nullptr};

        //! to be connected to \ref halui.spindle.is-on
        hal_bit_t  * spindleIsOn{nullptr};
        //! to be connected to \ref halui.spindle-override.value
        hal_float_t* spindleOverrideValue{nullptr};
        //! To be connected to an encoded and correctly scaled value of an spindle feedback signal.
        //! See also \ref encoder and \ref scale.
        hal_float_t* spindleSpeedAbsRpm{nullptr};

        //! to be connected to \ref motion.current-vel
        hal_float_t* feedSpeedUps{nullptr};
        //! to be connected to \ref halui.feed-override.value
        hal_float_t* feedOverrideValue{nullptr};

        //! the minimum feed-override value when in Lead-mode, usually same as [DISPLAY]MIN_FEED_OVERRIDE
        hal_float_t* feedOverrideMinValue{nullptr};
        //! the maximum feed-override value when in Lead-mode, usually same as [DISPLAY]MAX_FEED_OVERRIDE
        hal_float_t* feedOverrideMaxValue{nullptr};

        //! to be connected to \ref halui.program.is-running
        hal_bit_t* isProgramRunning{nullptr};
        //! to be connected to \ref halui.program.is-paused
        hal_bit_t* isProgramPaused{nullptr};
        //! to be connected to \ref halui.program.is-idle
        hal_bit_t* isProgramIdle{nullptr};

        //! to be connected to \ref halui.mode.is-auto
        hal_bit_t* isModeAuto{nullptr};
        //! to be connected to \ref halui.mode.is-joint
        hal_bit_t* isModeJoint{nullptr};
        //! to be connected to \ref halui.mode.is-manual
        hal_bit_t* isModeManual{nullptr};
        //! to be connected to \ref halui.mode.is-mdi
        hal_bit_t* isModeMdi{nullptr};

        //! to be connected to \ref halui.estop.is-activated
        hal_bit_t* isEmergencyStop{nullptr};

        //! to be connected to \ref halui.machine.is-on
        hal_bit_t* isMachineOn{nullptr};

        In();
    };

    struct Out
    {
    public:
        hal_bit_t* button_pin[64] = {0};

        //! to be connected to \ref axis.0.jog-counts
        hal_s32_t* axisXJogCounts{nullptr};
        //! to be connected to \ref axis.1.jog-counts
        hal_s32_t* axisYJogCounts{nullptr};
        //! to be connected to \ref axis.2.jog-counts
        hal_s32_t* axisZJogCounts{nullptr};
        //! to be connected to \ref axis.3.jog-counts
        hal_s32_t* axisAJogCounts{nullptr};
        //! to be connected to \ref axis.4.jog-counts
        hal_s32_t* axisBJogCounts{nullptr};
        //! to be connected to \ref axis.5.jog-counts
        hal_s32_t* axisCJogCounts{nullptr};

        //! to be connected to \ref axis.0.jog-enable
        hal_bit_t* axisXJogEnable{nullptr};
        //! to be connected to \ref axis.1.jog-enable
        hal_bit_t* axisYJogEnable{nullptr};
        //! to be connected to \ref axis.2.jog-enable
        hal_bit_t* axisZJogEnable{nullptr};
        //! to be connected to \ref axis.3.jog-enable
        hal_bit_t* axisAJogEnable{nullptr};
        //! to be connected to \ref axis.4.jog-enable
        hal_bit_t* axisBJogEnable{nullptr};
        //! to be connected to \ref axis.5.jog-enable
        hal_bit_t* axisCJogEnable{nullptr};

        //! to be connected to \ref axis.0.jog-scale
        hal_float_t* axisXJogScale{nullptr};
        //! to be connected to \ref axis.1.jog-scale
        hal_float_t* axisYJogScale{nullptr};
        //! to be connected to \ref axis.2.jog-scale
        hal_float_t* axisZJogScale{nullptr};
        //! to be connected to \ref axis.3.jog-scale
        hal_float_t* axisAJogScale{nullptr};
        //! to be connected to \ref axis.4.jog-scale
        hal_float_t* axisBJogScale{nullptr};
        //! to be connected to \ref axis.5.jog-scale
        hal_float_t* axisCJogScale{nullptr};

        //! to be connected to \ref axis.0.jog-vel-mode
        hal_bit_t* axisXSetVelocityMode{nullptr};
        //! to be connected to \ref axis.1.jog-vel-mode
        hal_bit_t* axisYSetVelocityMode{nullptr};
        //! to be connected to \ref axis.2.jog-vel-mode
        hal_bit_t* axisZSetVelocityMode{nullptr};
        //! to be connected to \ref axis.3.jog-vel-mode
        hal_bit_t* axisASetVelocityMode{nullptr};
        //! to be connected to \ref axis.4.jog-vel-mode
        hal_bit_t* axisBSetVelocityMode{nullptr};
        //! to be connected to \ref axis.5.jog-vel-mode
        hal_bit_t* axisCSetVelocityMode{nullptr};

        //! to be connected to \ref halui.spindle.stop
        hal_bit_t* spindleStop{nullptr};

        //! to be connected to \ref halui.spindle.forward
        hal_bit_t* spindleDoRunForward{nullptr};
        //! to be connected to \ref halui.spindle.reverse
        hal_bit_t* spindleDoRunReverse{nullptr};

        hal_bit_t* feedValueSelected0_001{nullptr};
        hal_bit_t* feedValueSelected0_01{nullptr};
        hal_bit_t* feedValueSelected0_1{nullptr};
        hal_bit_t* feedValueSelected1_0{nullptr};

        //! to be connected to \ref  \ref halui.feed-override.scale
        hal_float_t* feedOverrideScale{nullptr};
        //! to be connected to \ref halui.feed-override.direct-value
        hal_bit_t  * feedOverrideDirectValue{nullptr};
        //! to be connected to \ref halui.feed-override.count-enable
        hal_bit_t  * feedOverrideCountEnable{nullptr};
        //! to be connected to \ref halui.feed-override.counts
        hal_s32_t  * feedOverrideCounts{nullptr};
        //! to be connected to \ref halui.feed-override.decrease
        hal_bit_t  * feedOverrideDecrease{nullptr};
        //! to be connected to \ref halui.feed-override.increase
        hal_bit_t  * feedOverrideIncrease{nullptr};

        //! to be connected to halui.spindle.decrease
        hal_bit_t* spindleDoDecrease{nullptr};
        //! to be connected to halui.spindle.increase
        hal_bit_t* spindleDoIncrease{nullptr};
        //! to be connected to halui.spindle-override.decrease
        hal_bit_t* spindleOverrideDoDecrease{nullptr};
        //! to be connected to halui.spindle-override.increase
        hal_bit_t* spindleOverrideDoIncrease{nullptr};

        //! to be connected to \ref halui.jog-speed
        hal_float_t* jogSpeedValue{nullptr};

        //! to be connected to \ref halui.home-all
        hal_bit_t* homeAll{nullptr};

        //!to be connected to \ref halui.joint.N.select
        hal_bit_t* jointXSelect{nullptr};
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t* jointYSelect{nullptr};
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t* jointZSelect{nullptr};
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t* jointASelect{nullptr};
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t* jointBSelect{nullptr};
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t* jointCSelect{nullptr};

        //! reflects the pendant's idle state
        hal_bit_t* isPendantSleeping{nullptr};
        //! reflects pendant's connectivity
        hal_bit_t* isPendantConnected{nullptr};

        //! to be connected to \ref halui.program.run
        hal_bit_t* doRunProgram{nullptr};
        //! to be connected to \ref halui.program.pause
        hal_bit_t* doPauseProgram{nullptr};
        //! to be connected to \ref halui.program.resume
        hal_bit_t* doResumeProgram{nullptr};
        //! to be connected to \ref halui.program.stop
        hal_bit_t* doStopProgram{nullptr};

        //! to be connected to \ref halui.mode.auto
        hal_bit_t* doModeAuto{nullptr};
        //! to be connected to \ref halui.mode.joint
        hal_bit_t* doModeJoint{nullptr};
        //! to be connected to \ref halui.mode.manual
        hal_bit_t* doModeManual{nullptr};
        //! to be connected to \ref halui.mode.mdi
        hal_bit_t* doModeMdi{nullptr};

        //! to be connected to \ref halui.estop.activate
        hal_bit_t* doEmergencyStop{nullptr};
        //! to be connected to \ref halui.estop.reset
        hal_bit_t* resetEmergencyStop{nullptr};

        //! to be connected to \ref halui.machine.on
        hal_bit_t* doMachineOn{nullptr};
        //! to be connected to \ref halui.machine.off
        hal_bit_t* doMachineOff{nullptr};

        Out();
    };

    In  in;
    Out out;

    WhbHalMemory();
    ~WhbHalMemory();
};

// ----------------------------------------------------------------------

class WhbVelocityComputation
{
public:
    hal_s32_t      last_jog_counts;
    struct timeval last_tv;
    WhbVelocityComputation();
};

// ----------------------------------------------------------------------

//! HAL and related parameters
class WhbHal
{
public:
    WhbHalMemory* memory;
    std::map <std::string, size_t> mButtonNameToIdx;
    WhbVelocityComputation         velocityComputation;

    WhbHal();
    ~WhbHal();
    //! Initializes HAL memory and pins according to simulation mode. Must not be called more than once.
    //! If \ref mIsSimulationMode is true heap memory will be used, shared HAL memory otherwise.
    //! \ref setIsSimulationMode() must be set before accordingly
    void init(const WhbSoftwareButton* softwareButtons, const WhbKeyCodes& codes);
    bool isSimulationModeEnabled() const;
    //! indicates the program has been invoked in hal mode or normal
    void setSimulationMode(bool isSimulationMode);
    int getHalComponentId() const;
    //hal_bit_t* getButtonHalBit(size_t pinNumber);
    const char* getHalComponentName() const;
    //! Enables verbose hal output.
    //! \param enable true to enable hal messages, disable otherwise
    void setEnableVerbose(bool enable);
    //! If set indicates that no other axis is active.
    //! \param enabled true if no axis is active, false otherwise
    void setNoAxisActive(bool enabled);
    //! Sets the A axis to active or disables the same.
    //! \param enabled true if axis should be the one and only active
    void setAxisXActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisYActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisZActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisAActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisBActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisCActive(bool enabled);

    //! Sets the new feed rate. The step mode must be set accordingly.
    //! \param feedRate the new feed rate independent of step mode
    void setStepSize(const hal_float_t& feedRate);
    //! If lead is active.
    void setLead();
    //! Sets the hal state of the respective pin (reset). Usually called in case the reset
    //! button is pressed or released. The pin should be connected to \ref halui.estop.activate.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \param pinNumber The pin number in \ref WhbHalMemory as registered in
    //! \ref WhbHal::halInit(const WhbSoftwareButton* , size_t , const WhbKeyCodes&)
    //! \sa doEmergencyStop
    void setReset(bool enabled);
    //! \sa setReset(bool, size_t)
    void setStop(bool enabled);
    //! Toggles the start/pause/resume states.
    //! \param enabled true if start/resume is requested, false otherwise
    //! \param pinNumber \sa setReset(bool, size_t)
    void setStart(bool enabled);

    //! \sa setReset(bool, size_t)
    void setFeedPlus(bool enabled);
    //! \sa setReset(bool, size_t)
    void setFeedMinus(bool enabled);
    //! Returns the current feed override value.
    //! \return the current feed override value v: 0 <= v <= 1
    hal_float_t getFeedOverrideValue() const;
    // TODO: fix doxygen
    hal_float_t getFeedOverrideMinValue() const;
    // TODO: fix doxygen
    hal_float_t getFeedOverrideMaxValue() const;
    // TODO: fix doxygen
    //! \xrefitem HalMemory::Out::feedOverrideCounts setter
    void setFeedOverrideCounts(hal_s32_t counts);
    //! \xrefitem HalMemory::Out::feedOverrideScale setter
    void setFeedOverrideScale(hal_float_t scale);
    //! \xrefitem HalMemory::Out::feedOverrideDirectValue setter
    void setFeedOverrideCountEnable(bool enabled);
    //! \xrefitem HalMemory::Out::feedOverrideDirectValue setter
    void setFeedOverrideDirectValue(bool enabled);
    //! Returns the feed speed.
    //! \return the feed speed in unis per second
    hal_float_t getFeedUps() const;

    void setFeedValueSelected0_001(bool selected);
    void setFeedValueSelected0_01(bool selected);
    void setFeedValueSelected0_1(bool selected);
    void setFeedValueSelected1_0(bool selected);

    //! Returns the spindle speed.
    //! \return the spindle speed in rounds per second
    hal_float_t getSpindleSpeedAbsRpm() const;
    //! \sa setReset(bool, size_t)
    void setSpindlePlus(bool enabled);
    //! \sa setReset(bool, size_t)
    void setSpindleMinus(bool enabled);
    //! \sa setReset(bool, size_t)
    void setFunction(bool enabled);
    //! \sa setReset(bool, size_t)
    void setMachineHome(bool enabled);
    //! \sa setReset(bool, size_t)
    void setSafeZ(bool enabled);
    //! \sa setReset(bool, size_t)
    void setWorkpieceHome(bool enabled);
    //! \sa setReset(bool, size_t)
    void toggleSpindleDirection(bool isButtonPressed);
    //! \sa setReset(bool, size_t)
    void toggleSpindleOnOff(bool isButtonPressed);
    //! \sa setReset(bool, size_t)
    void setProbeZ(bool enabled);
    //! \sa setReset(bool, size_t)
    void setContinuousMode(bool enabled);
    //! \sa setReset(bool, size_t)
    void setStepMode(bool enabled);
    //! Sets the hal state of the macro pin. Usually called in case the macro
    //! button is pressed or released. A macro button can be any button
    //! when pressed together with the modifier key.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \param pinNumber The pin number in \ref WhbHalMemory.
    //! \sa setReset(bool, size_t)
    void setMacro1(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro2(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro3(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro4(bool enabled);
    //! \sa setMacro1(bool)
    void setMacro5(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro6(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro7(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro8(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro9(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro10(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro11(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro12(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro13(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro14(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro15(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro16(bool enabled);

    /**
     * Writes counts to each axis' count.
     * \param counts value to propagate to each axis
     */
    void setJogCounts(const HandWheelCounters& counters);

    //! Returns the axis position.
    //! \param absolute true absolute, false relative
    //! \return the absolute or relative position in machine units
    hal_float_t getAxisXPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisYPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisZPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisAPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisBPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisCPosition(bool absolute) const;

private:
    bool mIsSimulationMode;
    const char* mName;
    const char* mComponentPrefix;
    int          mHalCompId;
    std::ostream mDevNull;
    std::ostream* mHalCout;
    HandwheelStepmodes::Mode mStepMode;
    bool                     mIsSpindleDirectionForward{true};

    //! //! Allocates new hal_bit_t pin according to \ref mIsSimulationMode. If \ref mIsSimulationMode then
    //! mallocs memory, hal_pin_bit_new allocation otherwise.
    //! \param pin_name the pin name when registered to hal
    //! \param ptr will point to the allocated memory
    //! \param s size in bytes
    //! \return != 0 on error, 0 otherwise
    int newSimulatedHalPin(char* pin_name, void** ptr, int s);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalFloat(hal_pin_dir_t direction, hal_float_t** ptr, int componentId, const char* fmt, ...);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalSigned32(hal_pin_dir_t direction, hal_s32_t** ptr, int componentId, const char* fmt, ...);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalUnsigned32(hal_pin_dir_t direction, hal_u32_t** ptr, int componentId, const char* fmt, ...);
    //! \param direction module input or output
    //! \param ptr will point to the allocated memory
    //! \param componentId hal id
    //! \param fmt the pin name when registered to hal
    //! \param ... va args
    //! \return != 0 on error, 0 otherwise
    int newHalBit(hal_pin_dir_t direction, hal_bit_t** ptr, int componentId, const char* fmt, ...);
    //! allocates new hal pin according to \ref mIsSimulationMode
    //! \param pin pointer reference to the memory to be fred
    //! \post pin == nullptr
    void freeSimulatedPin(void** pin);
    //! \param enabled the new pin value
    //! \param pinNumber the pin number to set the value
    //! \param pinName arbitrary name for logging
    void setPin(bool enabled, size_t pinNumber, const char* pinName);
    //! \sa setPin(bool, size_t, const char*)
    void setPin(bool enabled, const char* pinName);
    //! Toggles program states; running, paused, resume.
    //! Should be called each time after setStart(true) (\sa setStart(bool)) to stay in sync.
    void toggleStartResumeProgram();

    void clearStartResumeProgramStates();

    void enableManualMode(bool isRisingEdge);

    void enableMdiMode(bool isRisingEdge);
};
}
