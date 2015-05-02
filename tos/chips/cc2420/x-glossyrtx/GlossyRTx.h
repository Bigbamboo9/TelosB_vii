/* -------------------------- Clock Capture ------------------------- */
/**
 * \brief Capture next low-frequency clock tick and DCO clock value at that instant.
 * \param t_cap_h variable for storing value of DCO clock value
 * \param t_cap_l variable for storing value of low-frequency clock value
 */
#define CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l) do {\
                /* Enable capture mode for timers B6 and A2 (ACLK) */\
                TBCCTL6 = CCIS0 | CM_POS | CAP | SCS; \
                TACCTL2 = CCIS0 | CM_POS | CAP | SCS; \
                /* Wait until both timers capture the next clock tick */\
                while (!((TBCCTL6 & CCIFG) && (TACCTL2 & CCIFG))); \
                /* Store the capture timer values */\
                t_cap_h = TBCCR6; \
                t_cap_l = TACCR2; \
                /* Disable capture mode */\
                TBCCTL6 = 0; \
                TACCTL2 = 0; \
} while (0)
