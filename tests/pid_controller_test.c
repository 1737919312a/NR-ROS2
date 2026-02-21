#include "pid_controller.h"

#include <math.h>
#include <stdio.h>

static int test_pid_compute_with_zero_ki_no_nan(void) {
    PIDController pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, -10.0f, 10.0f);

    float output = pid_compute(&pid, 100.0f, 0.0f, 0.1f);
    if (!isfinite(output)) {
        fprintf(stderr, "output is not finite when ki=0\n");
        return 1;
    }
    if (fabsf(output - 10.0f) > 1e-5f) {
        fprintf(stderr, "expected saturated output 10.0, got %.6f\n", output);
        return 1;
    }
    return 0;
}

static int test_diff_drive_straight_line(void) {
    DiffDriveKinematics kin;
    diff_drive_init(&kin, 0.2f, 0.03f);

    float left = 0.0f, right = 0.0f;
    diff_drive_twist_to_wheels(&kin, 0.5f, 0.0f, &left, &right);

    if (fabsf(left - right) > 1e-6f) {
        fprintf(stderr, "expected same wheel speed for straight motion, got left=%.6f right=%.6f\n", left, right);
        return 1;
    }
    return 0;
}

int main(void) {
    int failed = 0;
    failed += test_pid_compute_with_zero_ki_no_nan();
    failed += test_diff_drive_straight_line();

    if (failed != 0) {
        fprintf(stderr, "pid_controller_test failed with %d failing checks\n", failed);
        return 1;
    }

    printf("pid_controller_test passed\n");
    return 0;
}
