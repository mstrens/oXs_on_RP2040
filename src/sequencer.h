#pragma once

#include <stdint.h>
#include "param.h"

/*
enum SEQ_OUTPUT_TYPE {
    SERVO = 0,
    ANALOG = 1
};



// max 16X pin, type (SERVO/ANALOG) , clock ,channel ,default value
#define SEQ_DEFINITIONS {0,SERVO,200,3, -100}, {4,ANALOG,60,7, 0}


enum CH_RANGE {
    m100 = -100, m75 = -75, m50 = -50, m25 = -25, m0 = 0,
    p0 = 0, p25 = 25 , p50 = 50, p75 = 75, p100 = 100 , dummy = 127
};

struct SEQ_STEP {
    CH_RANGE chRange {}; // channel value (converted in range) to activate this seq
    uint8_t smooth {};
    int8_t value {} ;
    uint8_t keep {} ;
};

#define SEQ_STEPS   {m75,0,10,255},\
                    {p50,3,50,5},{p50,5,100,5},\
                    {p100,10,100,3},{p100,10,0,3},\
                    \
                    {p50,10,100,3},{p50,10,0,3},\
                    {p100,10,100,3},{p100,0,10,10}
                        
*/
enum SEQ_STATE {
    STOPPED = 0,
    SMOOTHING,
    WAITING
};

struct SEQ_DATA {             // one set of fields per sequencer= per gpio (so 16)
    uint16_t stepStartAtIdx ; // this is filled during setup
    uint16_t stepEndAtIdx ; // this is filled during setup
    SEQ_STATE state;        // Stopped, SMoothing, Waiting
    uint16_t currentChValueUs; // last channel value used to activated this sequence (in PWM us)
    CH_RANGE lastreceivedRange;   // range (-100, -75 , -25, ...+75, +100 , dummy) last range received from handset (even if not defined in the sequence)
    uint16_t firstStepIdx;   // first step to be played for current sequence
    uint16_t currentStepIdx; //step being played 
    uint16_t delayedStepIdx; // step to be played at the end of current step
    uint32_t smoothUpToMs;     // smoothing must end at this ms
    //uint32_t lastActionAtMs; // when last output update has been done (used for smoothing to calculate next value)
    uint32_t nextActionAtMs; // when an action has to be taken (increment smoothing or next step)
    int8_t lastOutputVal;    // last value that has been applied on output pin (in range -100/100).
    int smoothOnMs ; // total time foreseen for smoothing = setps[].smooth * seq[].clocks
    uint32_t smoothStartAtMs ; // timestamp when smoothing start
    uint8_t smoothStartValue ;     // value when smoothing starts
    int smoothGapValue ;       // gap between end and begin values  
};

//void setupSequencer(); // check the parameters and fill seqDatas
uint16_t searchSeq( uint8_t sequencer , CH_RANGE searchRange);
bool isSeqChannelChanged (uint8_t sequencer); // when true is returned, it means that there is a next step (stored in nextPossibleStepIdx) 
void startNewSeq(uint8_t sequencer , uint16_t stepIdx); // switch to the specified step into the specified sequencer
void startNewStep(uint8_t sequencer , uint16_t stepIdx); // start a new step
void sequencerLoop();

void updateSeqOutput(uint8_t sequencer, uint16_t stepIdx, int8_t outputVal);
void nextAction(uint8_t sequencer);
void printSequencerStatus();
bool nextStepExist(uint8_t sequencer, uint16_t currentStepIdx); // return true when there a next step in the same sequence

void nextSimuSeqChVal();


// imagine SEQ=[3 0 100 15 -100 -100 +90] (-100 R O) {0 50 3} {10 100 2} (100 P) {0 -100 10} 
//             [4 1 500 16 0 0 100 ] (-100) {0 0 20} (-30) {0 100 1} (70 R) {0 100 20} {5 20 10}

// in param.cpp we will fill some tables:

//SEQUENCER seq :                          // filled in param.cpp
//    uint8_t version = SEQUENCER_VERSION; = fixed value (to check config)
//    uint8_t defsMax = 0;                 = number of sequencers =>                         2
//    SEQ_DEF defs[16] ;                   = definitions of the sequencers                   (see below)
//    uint16_t sequencesMax = 0;           = total number of sequences (in all sequencers)   5
//    uint16_t stepsMax = 0;               = total number of steps (in all sequencers)       7
//    SEQ_STEP steps[SEQUENCER_MAX_NUMBER_OF_STEPS];  = definitions of all sequences and all steps  (see below)
//
//SEQ_DEF defs[16] ;                   = definitions of the sequencers; one record per sequencer
//    uint8_t pin {}; // pin
//    SEQ_OUTPUT_TYPE type {}; // type 0= SERVO or 1 = ANALOG
//    uint32_t clockMs {};  // unit in msec
//    uint8_t channel {};   // channel used to control the sequencer
//    int8_t defValue {} ;  // default value (when no channel is yet received)
//    int8_t minValue ;
//    int8_t maxValue ;
//            pin   type     clock  channel  defValue  min     max
// so here:     3      0      100      15     -100    -100      90
// defs=        4      1      500      16        0       0     100
//
//SEQ_STEP steps[]               = one set of parameters per sequence and per step
//    CH_RANGE chRange {}; // channel value (converted in range) to activate this seq
//    uint8_t smooth {};
//    int8_t value {} ;
//    uint8_t keep {} ;
//    uint8_t nextSequencerBegin:1; // 1 means that this is the first step of a new sequencer
//    uint8_t nextSequenceBegin:1; // 1 means that this is the first step of a new sequence
//    uint8_t toRepeat:1; // 1 means that this is the sequence must be repeated when reaching the end
//    uint8_t neverInterrupted:1 ;// 1 means that this sequence may never be interrupted by another one
//    uint8_t priorityInterruptOnly:1 ; // 1 means that this sequence may only be interrupted by a sequence marqued as priority
//    uint8_t isPriority:1 ;         // 1 means that this sequence must always interrupt a sequence that may be interrupted
//    uint8_t reserver1:1;           // not used currently
//    uint8_t reserver2:1;           // not used currently
//          chRange  smooth  value  keep  nextS..erBegin  nextS..eBegin toRepeat neverInt. prior.Int.Only isPrio.
// so here     -100       0    50     3           1            1          1         0           1          0
//             -100      10   100     2           0            0          1         0           1          0
//              100       0  -100    10           0            1          0         0           0          1
//             -100       0     0    20           1            1          0         0           0          0
//              -30       0   100     1           0            1          0         0           0          0
//               70       0   100    20           0            1          1         0           0          0           
//               70       5    10    10           0            0          1         0           0          0
// imagine SEQ=[3 0 100 15 -100 -100 +90] (-100 R O) {0 50 3} {10 100 2} (100 P) {0 -100 10} 
//             [4 1 500 16 0 0 100 ] (-100) {0 0 20} (-30) {0 100 1} (70 R) {0 100 20} {5 20 10}
//
//SEQ_DATA seqData[16] : one set of fields per sequencer
//    uint16_t stepStartAtIdx ; // this is filled during setup
//    uint16_t stepEndAtIdx ; // this is filled during setup
//    SEQ_STATE state;        // Stopped, SMoothing, Waiting
//    uint16_t currentChValueUs; // last channel value used to activated this sequence (in PWM us)
//    CH_RANGE lastreceivedRange;   // range (-100, -75 , -25, ...+75, +100 , dummy) last range received from handset (even if not defined in the sequence)
//    uint16_t firstStepIdx;   // first step to be played for current sequence
//    uint16_t currentStepIdx; //step being played 
//    uint16_t delayedStepIdx; // step to be played at the end of current step
//    uint32_t smoothUpToMs;     // smoothing must end at this ms
//    //uint32_t lastActionAtMs; // when last output update has been done (used for smoothing to calculate next value)
//    uint32_t nextActionAtMs; // when an action has to be taken (increment smoothing or next step)
//    int8_t lastOutputVal;    // last value that has been applied on output pin (in range -100/100).
//    int smoothOnMs ; // total time foreseen for smoothing = setps[].smooth * seq[].clocks
//    uint32_t smoothStartAtMs ; // timestamp when smoothing start
//    uint8_t smoothStartValue ;     // value when smoothing starts
//    int smoothGapValue ;       // gap between end and begin values  
//        start   end  state  curChValUs  lastRecRange firststep currentStep delayedStep ....
//so here  0       2                                              0,1 or2 
//         3       6                                              3,4,5 or 6               