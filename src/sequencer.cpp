#include "pico/stdlib.h"
#include "stdio.h"
#include "config.h"
#include "sequencer.h"
#include "tools.h"
#include "sbus_out_pwm.h"
#include "param.h"

#define NO_SEQ 0xFFFF  
//#ifdef SEQ_DEFINITIONS
//    SEQ_DEF seqDefs[] = { SEQ_DEFINITIONS }; 
//    SEQ_STEP seqSteps[] = { SEQ_STEPS };
//    SEQ_DATA seqDatas[16];
//#else
//    SEQ_DEF seqDefs[] = { }; 
//    SEQ_STEP seqSteps[] = { };
    SEQ_DATA seqDatas[16];
//
//#endif
//uint8_t seqDefMax = sizeof(seqDefs) / sizeof(SEQ_DEF) ; 
//uint16_t seqStepsMax = sizeof(seqSteps) / sizeof(SEQ_STEP) ;
//int8_t seqMsg = 0; // 0= no seq, 1 = valid, negative = error
//int16_t seqMsgParam = 0; 
uint16_t nextPossibleStepIdx = NO_SEQ;

//bool sequencerIsValid = false;

extern uint16_t rcSbusOutChannels[16];

extern uint32_t lastRcChannels;


extern SEQUENCER seq;

uint32_t currentSeqMillis ;
uint16_t currentChannelValue; 

//#define DEBUG_SIMULATE_SEQ_RC_CHANNEL
#define DEBUG_SIMULATE_SEQ_RC_CHANNEL_m100 191
#define DEBUG_SIMULATE_SEQ_RC_CHANNEL_STEP 200
#define DEBUG_SIMULATE_SEQ_INTERVAL 1000 // increase every 10 sec

uint16_t simuSeqChannelValue = DEBUG_SIMULATE_SEQ_RC_CHANNEL_m100;

    
void nextSimuSeqChVal(){
        simuSeqChannelValue = simuSeqChannelValue + DEBUG_SIMULATE_SEQ_RC_CHANNEL_STEP;
        if (simuSeqChannelValue > FROM_SBUS_MAX) simuSeqChannelValue = DEBUG_SIMULATE_SEQ_RC_CHANNEL_m100; 
        printf("chan=%i seqMax=%i\n",  (int) simuSeqChannelValue, (int) seq.defsMax);
}

void sequencerLoop(){
    // for each sequencer
    //     check if channel value changed and if so, if it match a valid sequence
    //     if true , apply new sequence from first step      
    //     else (if channel value is the same or does not match) : continue what means
    //          if next action is reached, apply next action (depend on state, ...)          
    //          else do nothing
    #ifdef DEBUG_SIMULATE_SEQ_RC_CHANNEL
    //static uint32_t lastSimuSeqMs = 0;
    //static uint16_t simuSeqChannelValue = DEBUG_SIMULATE_SEQ_RC_CHANNEL_m100;
    //if ( (millisRp() - lastSimuSeqMs) > DEBUG_SIMULATE_SEQ_INTERVAL ){
    //    lastSimuSeqMs = millisRp();
    //    simuSeqChannelValue = simuSeqChannelValue + DEBUG_SIMULATE_SEQ_RC_CHANNEL_STEP;
    //    if (simuSeqChannelValue > FROM_SBUS_MAX) simuSeqChannelValue = DEBUG_SIMULATE_SEQ_RC_CHANNEL_m100; 
    //    printf("chan=%i seqMax=%i\n",  (int) simuSeqChannelValue, (int) seqDefMax);
    //} 
    #endif    
    //if ( sequencerIsValid == false) return; // do nothing when there is no valid sequencer
    if ( ! lastRcChannels) return ;   // skip if we do not have last channels
    currentSeqMillis =  millisRp(); 
    for (uint8_t seqIdx = 0; seqIdx < seq.defsMax ; seqIdx++){ //same process for each sequencer
        if (seqDatas[seqIdx].nextActionAtMs == 0){ // this is when we are just starting; so we have to apply default value            
            // to do : apply seqDatas[seq].lastOutputVal); // take care to output the servo or analog value
           seqDatas[seqIdx].nextActionAtMs = 0XFFFFFFFF ; // wait for a channel change 
        }
        
        #ifndef DEBUG_SIMULATE_SEQ_RC_CHANNEL
        currentChannelValue = rcSbusOutChannels[seq.defs[seqIdx].channel - 1];
        #else
        currentChannelValue = simuSeqChannelValue; 
        #endif
        //if ( seq == 0 && currentChannelValue == 391) printf("ch is 391\n");
        if ( isSeqChannelChanged ( seqIdx)) {  // if channel value changed and is another range and steps are defined for it
            //printf("chan changed\n");
            startNewSeq(seqIdx, nextPossibleStepIdx); // activate new sequence
        } else { // channel did not changed 
            if (currentSeqMillis >= seqDatas[seqIdx].nextActionAtMs) {
                nextAction(seqIdx);
            }
        }    
    }
}

#define SBUS_AT_M100 191
#define SBUS_AT_P100 1792
#define SEQ_NUMBER_OF_INTERVALS 8
#define SEQ_SBUS_INTERVAL ( (SBUS_AT_P100 - SBUS_AT_M100) /SEQ_NUMBER_OF_INTERVALS )

void startNewSeq(uint8_t sequencer , uint16_t stepIdx){ // switch to the specified step into the specified sequencer
    // update 
    //seqDatas[sequencer].currentChValue = currentChannelValue ; 
    //printf("Start new sequencer %i at step %i\n",sequencer, stepIdx);
    seqDatas[sequencer].firstStepIdx = stepIdx; // store idx of the first step in this sequence
    startNewStep(sequencer, stepIdx); // start a new step
}

void startNewStep(uint8_t sequencer , uint16_t stepIdx){ // start a new step
    seqDatas[sequencer].currentStepIdx = stepIdx;
    
    //printf("Start new step for sequencer %i at step %i\n",sequencer, stepIdx);
    if (seq.steps[stepIdx].smooth == 0){ // When there is no smoothing delay, nextaction = currentSeqMillis + keep and stait = wait
        seqDatas[sequencer].state = WAITING;
        seqDatas[sequencer].nextActionAtMs = currentSeqMillis + (seq.defs[sequencer].clockMs * seq.steps[stepIdx].keep) ;
        seqDatas[sequencer].lastOutputVal = seq.steps[stepIdx].value; 
    } else {   // when there is a smooth parameter > 0, next action is 20 after current and state = SMOOTHING
        seqDatas[sequencer].state = SMOOTHING;
        //  lastOutputVal is not changed
        seqDatas[sequencer].smoothStartAtMs = currentSeqMillis ; 
        seqDatas[sequencer].smoothOnMs = (seq.defs[sequencer].clockMs * seq.steps[stepIdx].smooth); // smooth spread on xx ms
        seqDatas[sequencer].smoothGapValue = seq.steps[stepIdx].value - seqDatas[sequencer].lastOutputVal ; // total value to smooth
        seqDatas[sequencer].smoothUpToMs = currentSeqMillis + seqDatas[sequencer].smoothOnMs ; // smmothing part end at ms
        seqDatas[sequencer].smoothStartValue = seqDatas[sequencer].lastOutputVal ;  // save value when smoothing started
        seqDatas[sequencer].nextActionAtMs = currentSeqMillis + 20 ;
        if (seqDatas[sequencer].nextActionAtMs > seqDatas[sequencer].smoothUpToMs){
            seqDatas[sequencer].nextActionAtMs = seqDatas[sequencer].smoothUpToMs;
        }    
    }
    updateSeqOutput(sequencer, stepIdx, seqDatas[sequencer].lastOutputVal); // take care to output the servo or analog value
}

void updateSeqOutput(uint8_t sequencer, uint16_t stepIdx, int8_t outputVal){
    //if (sequencer == 0){
    //    printf("time=%u  seq=%i  range=%i  step=%i  val=%i state=%i\n",currentSeqMillis , sequencer , (int) seqDatas[sequencer].currentChValue,stepIdx , outputVal , (int) seqDatas[sequencer].state);
    //}
    int pwmValue;
    if (seq.defs[sequencer].type == SERVO) { // PWM signal varies from -100 to +100 and is mapped from 191 to 1792 usec
        pwmValue =  ((((int)outputVal - (int)-100) * 10) + 2000 + 1) / 2;
        if (pwmValue < 900) pwmValue = 900;
        if (pwmValue > 2200) pwmValue = 2200;   
    } else { // for analog we have to map 0 /100 to 0/20000  
        pwmValue = (int)outputVal * 200 ;
        if (pwmValue < 0) pwmValue = 0;
        if (pwmValue > 20000) pwmValue = 20000;
    }
    //printf("seq out val=%i pwm=%i\n", outputVal, pwmValue);
    applyPwmValue(seq.defs[sequencer].pin , (uint16_t) pwmValue); 
}

void nextAction(uint8_t sequencer){
    // if state = Smoothing
    //     if (nextActionMs + 20) > smoothUpToMs, change state to Waiting, change value , set nextaction = currentSeqMillis + (keep * clockMs) 
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
        if ( ( seqDatas[sequencer].nextActionAtMs + 20) >=  seqDatas[sequencer].smoothUpToMs) {
            // switch to WAITING state
            seqDatas[sequencer].state = WAITING;
            //seqDatas[sequencer].lastActionAtMs = currentSeqMillis;
            seqDatas[sequencer].nextActionAtMs = currentSeqMillis + ( seq.steps[currentStepIdx].keep * seq.defs[sequencer].clockMs);
            seqDatas[sequencer].lastOutputVal = seq.steps[currentStepIdx].value;
            updateSeqOutput( sequencer, currentStepIdx, seqDatas[sequencer].lastOutputVal);
        } else {
            // calculate next value to apply; state is unchanged
            //int currentGapMs = seqDatas[sequencer].smoothUpToMs - seqDatas[sequencer].lastActionAtMs ; 
            //int enlapsedMs =  currentSeqMillis - seqDatas[sequencer].lastActionAtMs ;
            //int currentGapValue = seq.steps[currentStepIdx].value - seqDatas[sequencer].lastOutputVal; 
            //seqDatas[sequencer].lastActionAtMs = currentSeqMillis;
            seqDatas[sequencer].nextActionAtMs = currentSeqMillis + 20;
            seqDatas[sequencer].lastOutputVal = seqDatas[sequencer].smoothStartValue +\
                ( ((int)  ( currentSeqMillis - seqDatas[sequencer].smoothStartAtMs)) *\
                     seqDatas[sequencer].smoothGapValue /  seqDatas[sequencer].smoothOnMs );
            //printf("At %i outVal=%i\n", currentSeqMillis , seqDatas[sequencer].lastOutputVal);
            updateSeqOutput( sequencer, currentStepIdx, seqDatas[sequencer].lastOutputVal);
        }
    } else { //state == waiting; end of pause is reached and so skip to next step (if any)
        if ( nextStepExist(sequencer, currentStepIdx) == true) { // return true when there a next step in the sequence, currentStepidx not updated and true is returned
            currentStepIdx++;
            startNewStep(sequencer, currentStepIdx); // start a new step with updated idx
        } else if ( seq.steps[currentStepIdx].keep == 255){ // we are on the last step and 255 means that we have to old the value
            //seqDatas[sequencer].lastActionAtMs = currentSeqMillis;
            seqDatas[sequencer].nextActionAtMs = 0XFFFFFFFF ; // new action in future 
            // keep last output val
        } else { // go back to first step of this sequence           
            startNewStep(sequencer , seqDatas[sequencer].firstStepIdx);
        }
    }
}

bool nextStepExist(uint8_t sequencer, uint16_t currentStepIdx){ // return true when there a next step in the same sequence
    if ( currentStepIdx == seqDatas[sequencer].stepEndAtIdx) return false; // we are on the last step of the last sequence 
    return seq.steps[currentStepIdx].chRange == seq.steps[currentStepIdx+1].chRange;    
    }

bool isSeqChannelChanged (uint8_t sequencer){ // when true is returned, it means that value changed and there is a next step (stored in nextPossibleStepIdx) 
    //if ( sequencer == 0 && currentChannelValue == 391) printf("sDcV=%i\n",seqDatas[sequencer].currentChValue);
    
    if ( currentChannelValue == seqDatas[sequencer].currentChValue ) {
        //if ( sequencer == 0 && currentChannelValue == 391) printf("case1\n");
        return false;
    } // when value is not exactly the same, find range (e.g. -100, -75, -50, -25, 0, 25, 50, 75, 100 )
    seqDatas[sequencer].currentChValue = currentChannelValue ;
    int rangeInt = (( ( (int) currentChannelValue - SBUS_AT_M100 )  + (SEQ_SBUS_INTERVAL / 2)) / SEQ_SBUS_INTERVAL)  ;
    CH_RANGE range =  (CH_RANGE) (rangeInt  * (200 / SEQ_NUMBER_OF_INTERVALS) - 100) ;
    //printf("range=%i\n", (int) range);
    //if ( sequencer == 0 && currentChannelValue == 391) printf("r=%i sSR=%i\n", (int) range, (int) rangeInt, (int) range);
    if ( seqDatas[sequencer].currentStepIdx != 0XFFFF ) { // do not check the current range when currentIdx is still a dummy value
        if (range == seq.steps[seqDatas[sequencer].currentStepIdx].chRange) { // flase when range does not change
            return false;
        }
    }    
    nextPossibleStepIdx = searchSeq( sequencer , range); // Search a sequence that match the range; searchSeq return NO_SEQ (=0XFFFF) if there is no step defined
    //if (nextPossibleStepIdx != NO_SEQ) {
    //    printf("At %i Sequencer %i chan= %i range= %i\n", (int) millisRp(), (int) sequencer , (int) currentChannelValue , (int) range);
    //}
    return ( nextPossibleStepIdx != NO_SEQ) ; 
}

uint16_t searchSeq( uint8_t sequencer , CH_RANGE searchRange){
    uint16_t idx = seqDatas[sequencer].stepStartAtIdx ;
    CH_RANGE loopRange  = seq.steps[idx].chRange ;
    CH_RANGE prevLoopRange = loopRange ;
    if (sequencer == 0 && ((int) searchRange) == -100 ){
        //printf("S=%i sr=%i lr=%i i=%i\n", (int) sequencer, (int) searchRange, (int) loopRange , (int) idx);
    }        
    while (idx < seq.stepsMax) {
        if ( ((int) loopRange) == ((int) searchRange)){
            //printf("S=%i sr=%i i=%i\n", (int) sequencer, (int) searchRange, (int) idx);
            return idx ;  
        } else if (loopRange > searchRange){
            //printf("case 2\n");
            return NO_SEQ ; 
        }    
        idx++;
        if (idx == seq.stepsMax) {
            //printf("case 3\n");
            return NO_SEQ ;
        } 
        prevLoopRange= loopRange ;
        loopRange = seq.steps[idx].chRange ;
        if (loopRange < prevLoopRange ) {
            //printf("case 4\n");
            return NO_SEQ ;
        }
    }
    //printf("case 5\n");
    return NO_SEQ;
}