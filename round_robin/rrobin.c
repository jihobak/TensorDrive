#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>


#define LOOP_COUNT 1000
pthread_t hpt, mpt, lpt;


int print_scheduling_info(pthread_t thread)
{
    struct sched_param param;
    int policy, rc;

    printf("Pid: %u \n", (unsigned int)pthread_self());
    return param.sched_priority;
}


void busywait(void)
{
    unsigned long i, t;
    for(i=0, i<900000; i++)
    {
        t=i;
        i=t;
    }
}


void *highprior_thread(void* arg)
{
    unsigned long i=1;

    pthread_t thread_id = pthread_self();
    printf("high priority(%x) thread start...\n", (unsigned int)thread_id);

    for(i=0; i<LOOP_COUNT; i++)
    {
        busywait();
        print_scheduling_info(thread_id);
    }

    printf("high priority(%x) thread end...\n", (unsigned int)thread_id);
    pthread_exit(0);
}


void *middleprior_thread(void *arg)
{
    unsigned long i=1;

    pthread_t thread_id = pthread_self();
    printf("middle priority(%x) thread start...\n", (unsigned int)thread_id);

    for(i=0; i<LOOP_COUNT;i++)
    {
        busywait();
        print_scheduling_info(thread_id);
    }
    
    printf("middle priority(%x) thread end...\n", (unsigned int)thread_id);
    pthread_exit(0);
}


void *lowprior_thread(void *arg)
{
    unsigned long i=1;

    pthread_t thread_id = pthread_self();
    printf("low priority(%x) thread start...\n", (unsigned int)thread_id);

    for(i=0; i<LOOP_COUNT; i++)
    {
        busywait();
        print_scheduling_info(thread_id);
    }

    printf("low priority(%x) thread end...\n", (unsigned int)thread_id);
    pthread_exit(0);
}


int main(int argc, char *argv[])
{
    struct sched_param my_param;
    pthread_attr_t hp_attr, mp_attr, lp_attr;
    int i;
    int min_priority = 90;
    int max_priority = 90;

    unsigned long set = 2; // processor 1


    i = sched_setaffinity(0, sizeof(set), &set);
    if(i !=0)
    {
        return 0;
    }


    /* MAIN_THREAD WITH LOW PRIORITY */
    min_priority = sched_get_priority_min(SCHED_RR);
    pthread_setschedparam(pthread_self(), SCHED_RR, &my_param);
    printf("min priority %d\n", min_priority);
    
    max_priority = sched_get_priority_max(SCHED_RR);
    printf("max priority %d\n", max_priority);

    print_scheduling_info(pthread_self());

    /* SCHEDULING POLICY AND PRIORITY FOR OTHER THREADS */

    pthread_attr_init(&lp_attr);
    pthread_attr_init(&mp_attr);
    pthread_attr_init(&hp_attr);

    set = 1; // PROCESSOR 0

    pthread_attr_setaffinity_np(&hp_attr, sizeof(set), &set);
    pthread_attr_setaffinity_np(&mp_attr, sizeof(set), &set);
    pthread_attr_setaffinity_np(&lp_attr, sizeof(set), &set);
    
    pthread_create(&lpt, &lp_attr, lowprior_thread, NULL);
    pthread_create(&mpt, &mp_attr, middleprior_thread, NULL);
    pthread_create(&hpt, &hp_attr, highprior_thread, NULL);

    pthread_join(htp, NULL);
    pthread_join(mpt, NULL);
    pthread_join(lpt, NULL);

    /* Clean up and exit */
    pthread_attr_destroy(&hp_attr);
    pthread_attr_destroy(&mp_attr);
    pthread_attr_destroy(&lp_attr);

    printf("main exiting\n");

    return 0;

}






