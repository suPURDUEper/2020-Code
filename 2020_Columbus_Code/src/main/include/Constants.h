#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

enum Constants 
{
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose
     * from 0,1,2, or 3. Only the first two (0,1) are visible in web-based configuration.
     */
    kSlotIdx = 0,

    /* Talon SRX/ Victor SPX will support multiple (cascaded) PID loops.
     * For now we just want the primary one.
     */
    kPIDLoopIdx = 0,

    /*
     * set to zero to skip waiting for confirmation, set to nonzero to wait
     * and report to DS if action fails
     */
    kTimeoutMs = 30
};

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#endif