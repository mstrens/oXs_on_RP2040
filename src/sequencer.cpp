#include "pico/stdlib.h"
#include "stdio.h"
#include "config.h"
#include "sequencer.h"
#include "tools.h"
#include "sbus_out_pwm.h"
#include "param.h"
#include "sport.h"
#include "hardware/watchdog.h"

#define NO_SEQ 0xFFFF  

#define SBUS_AT_M100 172
#define SBUS_AT_P100 1811
#define SEQ_NUMBER_OF_INTERVALS 40
//#define SEQ_SBUS_INTERVAL ( (SBUS_AT_P100 - SBUS_AT_M100) /SEQ_NUMBER_OF_INTERVALS )

//float seqSbusInterval = (float) (SBUS_AT_P100 - SBUS_AT_M100) /SEQ_NUMBER_OF_INTERVALS ; // interval in Sbus value between 2 intervals
float seqIntervalUs = (float) (2012-988) /SEQ_NUMBER_OF_INTERVALS ; // interval in pwm us value between 2 intervals


//#ifdef SEQ_DEFINITIONS
//    SEQ_DEF seqDefs[] = { SEQ_DEFINITIONS }; 
//    SEQ_STEP seqSteps[] = { SEQ_STEPS };
//    SEQ_DATA seqDatas[16];
//#else
//    SEQ_DEF seqDefs[] = { }; 
//    SEQ_STEP seqSteps[] = { };
SEQ_DATA seqDatas[16];          // store the current state of all sequencers
bool seqDatasToUpload = false;  // flag to say if seqDatas must be uploaded (after reset, SEQ or STEP command) or not (after ENTER); process is done in checkSequencer()
//
//#endif
//uint8_t seqDefMax = sizeof(seqDefs) / sizeof(SEQ_DEF) ; 
//uint16_t seqStepsMax = sizeof(seqSteps) / sizeof(SEQ_STEP) ;
//int8_t seqMsg = 0; // 0= no seq, 1 = valid, negative = error
//int16_t seqMsgParam = 0; 
uint16_t nextPossibleStepIdx = NO_SEQ;

//bool sequencerIsValid = false;

extern uint16_t rcChannelsUs[16];

extern uint32_t lastRcChannels;

extern field fields[NUMBER_MAX_IDX];  // list of all telemetry fields and parameters that can be measured (not only used by Sport)
extern CONFIG config; 

extern SEQUENCER seq;
extern uint16_t pwmTop;

uint32_t currentSeqMillis ;
uint16_t currentChannelValueUs; 

//#define DEBUG_SIMULATE_SEQ_RC_CHANNEL    // use the command N to let oXs apply the next value for the RC channel


//#define DEBUG_SIMULATE_SEQ_RC_CHANNEL_m100 FROM_SBUS_MIN  //172 // this is the value of sbus for -100
#define DEBUG_SIMULATE_SEQ_RC_CHANNEL_STEP 51.2 // = (2012-988)/20 = 51.2
#define DEBUG_SIMULATE_SEQ_INTERVAL 1000 // increase every 10 sec

//float simuSeqChannelValue = SBUS_AT_M100; //  172
float simuSeqChannelValueUs = 988;    // lower PWM value

void nextSimuSeqChVal(){
        simuSeqChannelValueUs = simuSeqChannelValueUs + DEBUG_SIMULATE_SEQ_RC_CHANNEL_STEP; // 
        if (simuSeqChannelValueUs > 2012) simuSeqChannelValueUs = 988; 
        printf("chan=%i seqMax=%i\n",  (int) simuSeqChannelValueUs, (int) seq.defsMax);
}

void sequencerLoop(){
    // for each sequencer
    //     check if channel value changed and if so, if it match a valid sequence
    //     if true , 
    //           if sequencer is stopped (state = STOPPED), new sequence starts immediately from first step
    //           else       
    //     else (if channel value is the same or does not match) : continue what means
    //          if next action is reached, apply next action (depend on state, ...)          
    //          else do nothing
    static uint32_t lastSeqTransmitMs = 0;
    uint32_t value1 = 0;        
    uint32_t value2 = 0;
    uint32_t code = 0;
    uint32_t mask = 0; 
    uint8_t shift = 0;
    int8_t lastOutput = 0;

    #ifdef DEBUG_SIMULATE_SEQ_RC_CHANNEL
    if (lastRcChannels == 0) lastRcChannels = 1; // force a dummy value to let sequencerLoop to run
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
    if ( seq.defsMax == 0) return ;   // skip if no sequencer is defined
    currentSeqMillis =  millisRp(); 
    for (uint8_t seqIdx = 0; seqIdx < seq.defsMax ; seqIdx++){ //same process for each sequencer
        if (seqDatas[seqIdx].nextActionAtMs == 0){ // this is when we are just starting; so we have to apply default value            
           updateSeqOutput(seqIdx, seqDatas[seqIdx].firstStepIdx , seqDatas[seqIdx].lastOutputVal);
           seqDatas[seqIdx].nextActionAtMs = 0XFFFFFFFF ; // wait for a channel change 
        }
        
        #ifndef DEBUG_SIMULATE_SEQ_RC_CHANNEL
        //currentChannelValue = rcSbusOutChannels[seq.defs[seqIdx].channel - 1];
        currentChannelValueUs = rcChannelsUs[seq.defs[seqIdx].channel - 1];
        
        #else
        //currentChannelValue = (uint16_t) simuSeqChannelValue; // is changed by sending a command N via USB
        currentChannelValueUs = (uint16_t) simuSeqChannelValueUs; // is changed by sending a command N via USB
        #endif
        if ( isSeqChannelChanged ( seqIdx)) {  // if channel value changed and is another range and steps are defined for it
                                               //  then nextPossibleStepIdx contains the step to be activated (or delayed)
            //printf("chan changed\n");
            if ( seqDatas[seqIdx].state == STOPPED ) {   // activate new sequence when no seq was running
                startNewSeq(seqIdx, nextPossibleStepIdx);
            } else {
                uint16_t firstStepIdx = seqDatas[seqIdx].firstStepIdx; 
                // if current sequence may be interrupted and new sequence is a priority one or current may be interrupted also by normal, activate new sequence
                if ( ( seq.steps[firstStepIdx].neverInterrupted == 0) && \
                       ( (seq.steps[nextPossibleStepIdx].isPriority == 1) || (seq.steps[firstStepIdx].priorityInterruptOnly == 0) )) {
                    startNewSeq(seqIdx, nextPossibleStepIdx);
                } else {
                // here we may not activate immediately the new sequence but we have to store it.
                    seqDatas[seqIdx].delayedStepIdx = nextPossibleStepIdx ; // store the step that can start only at the end of current sequence
                }
            }    
        } else { // channel did not changed 
            if (currentSeqMillis >= seqDatas[seqIdx].nextActionAtMs) {
                nextAction(seqIdx);
            }
        }    
    }
    #ifdef USE_RESERVE3_FOR_SPORT_FEEDBACK_FOR_SEQUENCES
    #define INTERVAL_BETWEEN_SEQUENCES_TRANSMIT 100 // in ms
    if ( currentSeqMillis > (lastSeqTransmitMs + INTERVAL_BETWEEN_SEQUENCES_TRANSMIT)) {
        lastSeqTransmitMs = currentSeqMillis;        
        value1 = 0b111111111111111111111111; // set the value to be transmitted when no value
        value2 = value1;
        for (uint8_t i = 0; i < seq.defsMax; i++){ 
            // for each sequencer, fill an array with a value 1 or 2 at the position of the gipo used by the sequencer
            // when gpio is above 16, mask the upper bits because there are only 16 pwm outputs
            code = 3 ;                      // value when PWM = 0
            lastOutput =  seqDatas[i].lastOutputVal;
            if (lastOutput < 0){
                if (lastOutput <= -75) code = 0;  // -100/-75
                else  if (lastOutput <= -40) code = 1; //-70/-40
                else code = 2;                         //-35/-5
            } else if (lastOutput >0) {
                if (lastOutput <= 35) code = 4;        // 5/35  
                else  if (lastOutput <= 70) code = 5;  //40/70
                else  if (lastOutput <= 100) code = 6;  //75/100
                else code = 7;                          //more than 100
            } 
            shift = ((7 - (seq.defs[i].pin & 0X07)) * 3);
            code =  code <<  shift  ;
            mask = ~((uint32_t) 0x00000007 << shift) ;  // set 3 bits to 0
            if ((seq.defs[i].pin & 0X08) == 0) {
                value1 &= mask ;  // set 3 bits to 0
                value1 |= code;              // set the value
            } else {
                value2 &= mask ;  // set 3 bits to 0
                value2 |= code;              // set the value
            }
            //printf("i=%i lo=%i s=%i c=%i m=%i v1=%i v2=%i\n" , i, lastOutput , shift, code , mask , value1 , value2); 
        }  // end for
        fields[RESERVE3].value = (int32_t) value1;
        fields[RESERVE3].available = true ;
        fields[RESERVE4].value = (int32_t) value2;
        fields[RESERVE4].available = true ;
        
        if ((fields[RESERVE3].onceAvailable == false) || (fields[RESERVE3].onceAvailable == false) ){
            fields[RESERVE3].onceAvailable = true;
            fields[RESERVE4].onceAvailable = true;
            if ( (config.protocol == 'S') || (config.protocol == 'F') ) { 
                calculateSportMaxBandwidth();
            }
        }
//  just to debug
/*
        printf("value 0..7=%i ", (int32_t) value1 ) ;
        for( uint8_t i = 0 ; i<8; i++) {
            printf(" %i ", (int8_t) ((value1 >> (21- i*3)) & 0X07 )); // group of  bits
        }
        printf("  value 8..15=%i ", (int32_t) value2  ) ;
        for( uint8_t i = 0 ; i<8; i++) {
            printf(" %i ", (int8_t) ((value2 >> (21- i*3)) & 0X07 )); // group of  bits
        }
        
        printf("\n");       
*/
    }
    #endif
}


void startNewSeq(uint8_t sequencer , uint16_t stepIdx){ // switch to the specified step into the specified sequencer
    // update 
    //seqDatas[sequencer].currentChValue = currentChannelValue ; 
    //printf("Start new sequencer %i at step %i\n",sequencer, stepIdx);
    seqDatas[sequencer].firstStepIdx = stepIdx; // store idx of the first step in this sequence
    seqDatas[sequencer].delayedStepIdx = 0xFFFF;  // reset index of a delayed step to a dummy value
    startNewStep(sequencer, stepIdx); // start a new step
}

void startNewStep(uint8_t sequencer , uint16_t stepIdx){ // start a new step
    seqDatas[sequencer].currentStepIdx = stepIdx;
    
    //printf("Start new step for sequencer %i at step %i\n",sequencer, stepIdx);
    if (seq.steps[stepIdx].smooth == 0){ // When there is no smoothing delay, nextaction = currentSeqMillis + keep and state = wait
        seqDatas[sequencer].state = WAITING;
        seqDatas[sequencer].nextActionAtMs = currentSeqMillis + (seq.defs[sequencer].clockMs * seq.steps[stepIdx].keep) ;
        if ( seq.steps[stepIdx].value != 127 ) {  // avoid to update PWM when new value == 127 (= stop at current position)
            seqDatas[sequencer].lastOutputVal = seq.steps[stepIdx].value;
        }     
    } else {   // when there is a smooth parameter > 0, next action is 20 ms after current and state = SMOOTHING
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
    if (seq.defs[sequencer].type == SERVO) { // PWM signal varies from -100 to +100 and is mapped from 191 to 1792 in sbus and to 988 to 2012 usec
        pwmValue =  (int)((((float)outputVal + 100.0) * 10.24) + 1977) / 2; // 1977 = 988*2+1
        if (pwmValue < 900) pwmValue = 900;
        if (pwmValue > 2200) pwmValue = 2200;   
    } else { // for analog we have to map 0...100 to 0...(pwmTop)  
        pwmValue = (int)outputVal * (int) pwmTop / 100 ;
        if (pwmValue < 0) pwmValue = 0;
        if (pwmValue > pwmTop) pwmValue = (int) pwmTop;
    }
    //printf("seq out val=%i pwm=%i\n", outputVal, pwmValue);
    applyPwmValue(seq.defs[sequencer].pin , (uint16_t) pwmValue); 
}

void nextAction(uint8_t sequencer){
    // if state = Smoothing
    //     if (nextActionMs + 20) > smoothUpToMs, change state to Waiting, change value , set nextaction = currentSeqMillis + (keep * clockMs) 
    //     else calculate next value to apply and keep state = smoothingcurrentStepIdx
    //else (state = waiting) perform nextstep()
    //     if there is a next step in current sequence
    //           change current stepidx
    //           start new sequence
    //     else
    //           if a delayed sequence exist, then start the delayed sequence            
    //           else if current step has the flag "Repeat", go back to the first step of the same sequence
    //           else set STATE=STOPPED.
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
        } else {         // we are at the end of a sequence
            if ( seqDatas[sequencer].delayedStepIdx != 0xFFFF){       // when a step was delayed, start new sequence
                startNewSeq(sequencer , seqDatas[sequencer].delayedStepIdx); 
            } else if ( seq.steps[currentStepIdx].toRepeat == 1) {    // when current sequence must be repeated, go to first step
                startNewStep(sequencer , seqDatas[sequencer].firstStepIdx);
            } else {
                seqDatas[sequencer].state = STOPPED;             // mark the sequence as STOPPED; so next change can start a new sequence immediately    
            }
        }
    }
}

bool nextStepExist(uint8_t sequencer, uint16_t currentStepIdx){ // return true when there a next step in the same sequence
    if ( currentStepIdx == seqDatas[sequencer].stepEndAtIdx) return false; // we are on the last step of the last sequence 
    return seq.steps[currentStepIdx].chRange == seq.steps[currentStepIdx+1].chRange;    
    }

bool isSeqChannelChanged (uint8_t sequencer){ // when true is returned, it means that value changed and there is a next step (stored in nextPossibleStepIdx) 
    //if ( sequencer == 0 && currentChannelValue == 391) printf("sDcV=%i\n",seqDatas[sequencer].currentChValue);
    
    if ( currentChannelValueUs == seqDatas[sequencer].currentChValueUs ) {
        //if ( sequencer == 0 && currentChannelValue == 391) printf("case1\n");
        return false;
    } // when value is not exactly the same, find range (e.g. -100, -90, -70,... 100 )
    seqDatas[sequencer].currentChValueUs = currentChannelValueUs ;
    //int rangeInt =  (int) (( ( (float) currentChannelValue - SBUS_AT_M100 )  + (seqSbusInterval / 2)) / seqSbusInterval)  ; // so normally 988=>0,, 988+89.1, ... 1, 2..
    int rangeInt =  (int) (( ( (float) currentChannelValueUs - 988 )  + (seqIntervalUs / 2)) / seqIntervalUs)  ; // so normally 988=>0,, 988+89.1, ... 1, 2..
    
    CH_RANGE range =  (CH_RANGE) (rangeInt  * (200 / SEQ_NUMBER_OF_INTERVALS) - 100) ;
    if (seqDatas[sequencer].lastreceivedRange == range) {   // when range did not change
        return false; 
    }
    seqDatas[sequencer].lastreceivedRange = range; // store the current range
    //printf("range=%i\n", (int) range);
    //if ( seqDatas[sequencer].currentStepIdx != 0XFFFF ) { // do not check the current range when currentIdx is still a dummy value
    //    if (range == seq.steps[seqDatas[sequencer].currentStepIdx].chRange) { // flase when range does not change
    //        return false;
    //    }
    //}    
    
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