#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#define N 256
#define THREADS 2

static int A[N][N], B[N][N], C[N][N];

static void *multiply(void *arg)
{
    int id = *(int *)arg;
    int start = id * N / THREADS;
    int end = (id + 1) * N / THREADS;

    for (int i = start; i < end; i++) {
        for (int j = 0; j < N; j++) {
            int sum = 0;
            for (int k = 0; k < N; k++)
                sum += A[i][k] * B[k][j];
            C[i][j] = sum;
        }
    }
    return NULL;
}

int main(void)
{
    pthread_t threads[THREADS];
    int ids[THREADS];

    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            A[i][j] = i + j;
            B[i][j] = i - j;
        }

    for (int i = 0; i < THREADS; i++) {
        ids[i] = i;
        if (pthread_create(&threads[i], NULL, multiply, &ids[i]) != 0) {
            fprintf(stderr, "Failed to create thread %d\n", i);
            return EXIT_FAILURE;
        }
    }

    for (int i = 0; i < THREADS; i++) {
        if (pthread_join(threads[i], NULL) != 0) {
            fprintf(stderr, "Failed to join thread %d\n", i);
            return EXIT_FAILURE;
        }
    }
    //int id = 0;
    //multiply(&id);

    printf("Result: %d\n", C[N - 1][N - 1]);
    return EXIT_SUCCESS;
}