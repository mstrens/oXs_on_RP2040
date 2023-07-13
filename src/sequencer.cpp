#include "pico/stdlib.h"
#include "stdio.h"
#include "config.h"
#include "sequencer.h"
#include "tools.h"

#define NO_SEQ 0xFFFF  
#ifdef SEQ_DEFINITIONS
    SEQ_DEF seqDefs[] = { SEQ_DEFINITIONS }; 
    SEQ_STEP seqSteps[] = { SEQ_STEPS };
    SEQ_DATA seqDatas[16];
#else
    SEQ_DEF seqDefs[] = { }; 
    SEQ_STEP seqSteps[] = { };
    SEQ_DATA seqDatas[16];

#endif
uint8_t seqDefMax = sizeof(seqDefs) / sizeof(SEQ_DEF) ; 
uint16_t seqStepsMax = sizeof(seqSteps) / sizeof(SEQ_STEP) ;
int8_t seqMsg = 0; // 0= no seq, 1 = valid, negative = error
int16_t seqMsgParam = 0; 
uint16_t nextPossibleStepIdx = NO_SEQ;

extern uint16_t rcSbusOutChannels[16];

void printSequencerStatus(){
    switch (seqMsg) {
        case 0 :
            printf("No sequencer defined\n");
            break;
        case 1 :
            printf("%i pins are controlled by a sequencer; setup is valid\n", seqDefMax);
            break;
        case -1 :
            printf("Error in sequencer steps: only one range for sequencers %i\n", seqMsgParam);
            break;
        case -2 :
            printf("Error in sequencer steps: to many sequencers detected (based on step breaks)\n");
            break;
        case -3 :
            printf("Error in sequencer steps: only one range at step %i\n", seqMsgParam);
            break;
        case -4 :
            printf("Error in sequencer steps: no enough steps defined \n", seqMsgParam);
    } 
}

void setupSequencer(){
    if ( (seqStepsMax == 0) || (seqDefMax == 0)) return ; // skip when sequencer are not defined
    uint8_t seqIdx = 0;  // index of current sequencer
    uint16_t stepIdx = 0;   // index of current step
    uint16_t prevStepIdx = 0;
    
    uint8_t rangeNumber = 1; // used to check that each sequencer has at least 2 sequence range
    uint32_t currentSeqMillis = millisRp();
    CH_RANGE prevRange = seqSteps[stepIdx].chRange; 
    seqDatas[seqIdx].stepStartAtIdx = stepIdx;
    seqDatas[seqIdx].state = STOPPED;
    seqDatas[seqIdx].currentChValue = 0; // use a dummy channel value in order to force a change when a channel value will be received from Rx
    seqDatas[seqIdx].currentStepIdx = stepIdx;
    seqDatas[seqIdx].lastActionAtMs = 0;
    seqDatas[seqIdx].lastOutputVal = seqDefs[seqIdx].defValue ; // set default value
    seqDatas[seqIdx].nextActionAtMs = 0; // 0 means that we still have to apply the default
    while (stepIdx < seqStepsMax) {
        if ( seqSteps[stepIdx].chRange > prevRange) rangeNumber++; // Count the number of range for this sequencer
        if ( seqSteps[stepIdx].chRange < prevRange ) {  // a new sequencer starts *****************
            if ( rangeNumber < 2) {
                seqMsg = -1;// "Error in sequencer steps: only one range for sequencers %i\n",(int) seqIdx+1 );
                seqMsgParam = seqIdx+1 ; 
                seqDefMax = 0;  // reset the number of sequencer
                return;
            }
            seqDatas[seqIdx].stepEndAtIdx = prevStepIdx;
            seqIdx++;
            if (seqIdx >= seqDefMax) {
                seqMsg = -2; //"Error in sequencer steps: to many sequencers detected (based on step breaks)\n");
                seqDefMax = 0;  // reset the number of sequencer
                return;
            }
            // initilize seqDatas for new sequencer
            seqDatas[seqIdx].stepStartAtIdx = stepIdx;
            seqDatas[seqIdx].state = STOPPED;
            seqDatas[seqIdx].currentChValue = 0; // use a dummy channel value in order to force a change when a channel value will be received from Rx
            seqDatas[seqIdx].currentStepIdx = stepIdx;
            seqDatas[seqIdx].lastActionAtMs = 0;
            seqDatas[seqIdx].lastOutputVal = seqDefs[seqIdx].defValue ; // set default value
            seqDatas[seqIdx].nextActionAtMs = 0; // 0 means that we have still to apply the default value      
            rangeNumber = 1; // restat a new counting
        }
        prevRange = seqSteps[stepIdx].chRange ; 
        prevStepIdx = stepIdx;
        stepIdx++;
    } // end while
    if ( rangeNumber < 2) {
                seqMsg = -3; //"Error in sequencer steps: only one range for step %i\n",(int) stepIdx+1 );
                seqMsgParam = stepIdx;
                seqDefMax = 0;  // reset the number of sequencer
                return;
            }
    if( (seqIdx + 1) != seqDefMax) {
        seqMsg = -4; //"Error in sequencer steps: no enough steps defined \n");
        seqDefMax = 0;  // reset the number of sequencer
        return;
    }
    seqDatas[seqIdx].stepEndAtIdx = prevStepIdx; // for the last sequencer, register the last valid stepIdx
            
    seqMsg =  1 ; // no error in sequencer detected
}

uint32_t currentSeqMillis ;
uint16_t currentChannelValue; 

void loopSequencer(){
    // for each sequencer
    //     check if channel value changed and if so, if it match a valid sequence
    //     if true , apply new sequence from first step      
    //     else (if channel value is the same or does not match) : continue what means
    //          if next action is reached, apply next action (depend on state, ...)          
    //          else do nothing
    if (seqDefMax == 0) return; // do nothing when there is no sequencer
    currentSeqMillis =  millisRp(); 
    for (uint8_t seq = 0; seq < seqDefMax ; seq++){ //same process for each sequencer
        if (seqDatas[seq].nextActionAtMs == 0){ // this is when we are just starting; so we have to apply default value            
            // to do : apply seqDatas[seq].lastOutputVal); // take care to output the servo or analog value
           seqDatas[seq].nextActionAtMs = 0XFFFFFFFF ; // wait for a channel change 
        }
        currentChannelValue = rcSbusOutChannels[seqDefs[seq].channel];
        if ( isSeqChannelChanged ( seq)) {  // if channel value changed and is another range and steps are defined for it
            startNewSeq(seq, nextPossibleStepIdx); // activate new sequence
        } else { // channel did not changed 
            if (currentSeqMillis >= seqDatas[seq].nextActionAtMs) {
                nextAction(seq);
            }
        }    
    }
}

#define SBUS_AT_0 ( (FROM_SBUS_MIN + FROM_SBUS_MAX) /2 )
#define SBUS_AT_100 1800
#define SEQ_NUMBER_OF_INTERVALS 8
#define SEQ_SBUS_INTERVAL ( (SBUS_AT_100 - SBUS_AT_0) /SEQ_NUMBER_OF_INTERVALS / 2 )

void startNewSeq(uint8_t sequencer , uint16_t stepIdx){ // switch to the specified step into the specified sequencer
    // update 
    seqDatas[sequencer].currentChValue = currentChannelValue ; 
    startNewStep(sequencer, stepIdx); // start a new step
}

void startNewStep(uint8_t sequencer , uint16_t stepIdx){ // start a new step
    seqDatas[sequencer].currentStepIdx = stepIdx;
    seqDatas[sequencer].lastActionAtMs = currentSeqMillis ; // 
    if (seqSteps[stepIdx].smooth == 0){ // When there is no smoothing delay, nextaction = currentSeqMillis + keep and stait = wait
        seqDatas[sequencer].state = WAITING;
        seqDatas[sequencer].nextActionAtMs = currentSeqMillis + (seqDefs[sequencer].clockMs * seqSteps[stepIdx].keep) ;
        seqDatas[sequencer].lastOutputVal = seqSteps[stepIdx].value; 
    } else {   // when there is a smooth parameter > 0, next action is 20 after current and state = SMOOTHING
        seqDatas[sequencer].state = SMOOTHING;
        //  lastOutputVal is not changed
        seqDatas[sequencer].smoothToMs = currentSeqMillis + (seqDefs[sequencer].clockMs * seqSteps[stepIdx].smooth) ;
        seqDatas[sequencer].nextActionAtMs = currentSeqMillis + 20 ;
        if (seqDatas[sequencer].nextActionAtMs > seqDatas[sequencer].smoothToMs){
            seqDatas[sequencer].nextActionAtMs = seqDatas[sequencer].smoothToMs;
        }    
    }
    updateSeqOutput(sequencer, stepIdx, seqDatas[sequencer].lastOutputVal); // take care to output the servo or analog value
}

void updateSeqOutput(uint8_t sequencer, uint16_t stepIdx, int8_t outputVal){
    printf("time=%u  seq=%i  step=%i  val=%i", currentSeqMillis , sequencer , stepIdx , outputVal );
}

void nextAction(uint8_t sequencer){
    // if state = Smoothing
    //     if (nextActionMs + 20) > smoothToMs, change state to Waiting, change value , set nextaction = currentSeqMillis + (keep * clockMs) 
    //     else calculate next value to apply and keep state = smoothing
    //else (state = waiting) perform nextstep()
    //     if there is a next step in current sequence
    //           change current stepidx
    //           start new sequence
    //     else
    //           if keep (of current step) == 255, keep current output value and set nextAction = 0XFFFFFFFF
    //           else go back to first step of same sequence 
    uint16_t currentStepIdx = seqDatas[sequencer].currentStepIdx ;
    if (seqDatas[sequencer].state == SMOOTHING){
        if ( ( seqDatas[sequencer].nextActionAtMs + 20) >=  seqDatas[sequencer].smoothToMs) {
            // switch to WAITING state
            seqDatas[sequencer].state = WAITING;
            seqDatas[sequencer].lastActionAtMs = currentSeqMillis;
            seqDatas[sequencer].nextActionAtMs = currentSeqMillis + ( seqSteps[currentStepIdx].keep * seqDefs[sequencer].clockMs);
            seqDatas[sequencer].lastOutputVal = seqSteps[currentStepIdx].value;
            updateSeqOutput( sequencer, currentStepIdx, seqDatas[sequencer].lastOutputVal);
        } else {
            // calculate next value to apply; state is unchanged
            int remainGapMs = currentSeqMillis - seqDatas[sequencer].lastActionAtMs ;
            int remainGapValue = seqSteps[currentStepIdx].value - seqDatas[sequencer].lastOutputVal; 
            seqDatas[sequencer].lastActionAtMs = currentSeqMillis;
            seqDatas[sequencer].nextActionAtMs = currentSeqMillis + 20;
            seqDatas[sequencer].lastOutputVal = seqDatas[sequencer].lastOutputVal + ( remainGapValue * 20 / remainGapMs );
            updateSeqOutput( sequencer, currentStepIdx, seqDatas[sequencer].lastOutputVal);
        }
    } else { //state == waiting; end of pause is reached and so skip to next step (if any)
        if ( nextStepExist(sequencer, currentStepIdx) == true) { // return true when there a next step in the sequence, currentStepidx not updated and true is returned
            currentStepIdx++;
            startNewStep(sequencer, currentStepIdx); // start a new step with updated idx
        } else if ( seqSteps[currentStepIdx].keep == 255){ // we are on the last step and 255 means that we have to old the value
            seqDatas[sequencer].lastActionAtMs = currentSeqMillis;
            seqDatas[sequencer].nextActionAtMs = 0XFFFFFFFF ; // new action in future 
            // keep last output val
        } else { // go back to first step of this sequence
            startNewStep(sequencer , seqDatas[sequencer].stepStartAtIdx);
        }
    }
}

bool nextStepExist(uint8_t sequencer, uint16_t currentStepIdx){ // return true when there a next step in the same sequence
    if ( currentStepIdx == seqDatas[sequencer].stepEndAtIdx) return false; // we are on the last step of the last sequence 
    return seqSteps[currentStepIdx].chRange != seqSteps[currentStepIdx+1].chRange;    
    }

bool isSeqChannelChanged (uint8_t sequencer){ // when true is returned, it means that value changed and there is a next step (stored in nextPossibleStepIdx) 
    if ( currentChannelValue == seqDatas[sequencer].currentChValue ) {
        return false;
    } // when value is not exactly the same, find range (e.g. -100, -75, -50, -25, 0, 25, 50, 75, 100 )
    CH_RANGE range =  (CH_RANGE)( ((( ( (int) currentChannelValue - SBUS_AT_0 )*100)  + (SEQ_SBUS_INTERVAL / 2)) / SEQ_SBUS_INTERVAL) * 200 / SEQ_SBUS_INTERVAL);
    if (range == seqSteps[seqDatas[sequencer].currentStepIdx].chRange ) {
        return false;
    }
    nextPossibleStepIdx = searchSeq( sequencer , range); // Search a sequence that match the range; searchSeq return NO_SEQ (=0XFFFF) if there is no step defined
    return ( nextPossibleStepIdx != NO_SEQ) ; 
}

uint16_t searchSeq( uint8_t sequencer , CH_RANGE searchRange){
    uint16_t idx = seqDatas[sequencer].stepStartAtIdx ;
    CH_RANGE loopRange  = seqSteps[idx].chRange ;
    CH_RANGE prevLoopRange = loopRange ;
    while (idx < seqStepsMax) {
        if ( loopRange == searchRange){
            return idx ;  
        } else if (loopRange < searchRange){
            return NO_SEQ ; 
        }    
        idx++;
        if (idx == seqStepsMax) {
            return NO_SEQ ;
        } 
        prevLoopRange= loopRange ;
        loopRange = seqSteps[idx].chRange ;
        if (loopRange < prevLoopRange ) {
            return NO_SEQ ;
        }
    }
    return NO_SEQ;
}