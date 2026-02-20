import sys
import numpy as np
import matplotlib.pyplot as plt

ACTUAL = np.array([31.87, 1.59344, 6.1089, 0.998314])  # Actual parameters used in simulation
TIME_CONSTANT = 1.0                                    # Time constant for thruster response
c1 = 0.16197                                            # Thrust coefficient (N/(RPS^2))   
GRAVITY = 9.8                                           # Gravity acceleration (m/s^2)

def get_data_from_npz(file_path, noisy=False):
    print(f"Reading {file_path}")
    d = np.load(file_path)
    
    if not noisy:
        t = d["A"][:, 0]
        RC = np.interp(t, d["Rc"][:, 0], d["Rc"][:, 1])
        V = np.interp(t, d["V"][:, 0], d["V"][:, 1])
        A = d["A"][:, 1]
        
        return t, RC, V, A
    else:
        u_times = d['u'][:, 0]
        acc = d['u'][:, 4:7]  # shape (N, 3)
        dvl_data = d['z1']
        rc = d['rc']

        # Get the gravity vector in the body frame
        assert d['x'].shape[1] == 26, "Unexpected x array shape"
        gravity = np.array([0, 0, GRAVITY])  # Gravity vector in world frame
        x_data = d['x'][:, 1:].reshape((-1,5,5))
        rot_matrices = x_data[:, 0:3, 0:3]  # Extract rotation matrices
        gravity_body = np.zeros((rot_matrices.shape[0], 3))
        for i in range(rot_matrices.shape[0]):
            gravity_body[i] = rot_matrices[i].T @ gravity  # Rotate gravity into body frame
                
        t2 = u_times  # x acceleration timestamps
        A2 = acc[:, 0] - gravity_body[:, 0]  # x acceleration
        V2 = np.interp(t2, dvl_data[:, 0], dvl_data[:, 1])  # x velocity
        RC2 = np.interp(t2, rc[:, 0], rc[:, 5])  
    
        return t2, RC2, V2, A2
        


def solve_ls(d, noisy=False):
    t, RC, V, A = get_data_from_npz(d, noisy=noisy)
    # Filter low-velocity samples
    mask = np.abs(V) > 0.001
    a, v, rc, t_m = A[mask], V[mask], RC[mask], t[mask]

    F1 = []
    U_Actual = []
    prev_rpms = 0.0
    prev_time = None
    
    # First order filter that maps commanded input to actual RPM
    for i, r in enumerate(rc):
        if prev_time is None:
            prev_time = t_m[i]
            F1.append(0.0)
            U_Actual.append(0.0)
            continue

        dt = t_m[i] - prev_time
        alpha = dt / (TIME_CONSTANT + dt)
        rpm = prev_rpms + alpha * (r - prev_rpms)
        prev_rpms = rpm
        prev_time = t_m[i]

        rps = rpm / 60
        f = c1 * rps * abs(rps)
        F1.append(f)
        U_Actual.append(rps)

    # F1 = m a + Dl v * e^(-3v) + Dq v|v| + c2 * delta * |v|
    U_Actual = np.asarray(U_Actual)
    X = np.column_stack([a, v * np.exp(-3 * np.abs(v)), v * np.abs(v), U_Actual * np.abs(v)])
    y = np.asarray(F1)

    params, res, rank, s = np.linalg.lstsq(X, y, rcond=None)

    print(f"Mass                    : {params[0]:.4f}")
    print(f"Lin Drag                : {params[1]:.4f}")
    print(f"Quad Drag               : {params[2]:.4f}")
    print(f"Thrust Degration Coeff  : {params[3]:.4f}")
    if res.size > 0:
        print(f"Residuals              : {res[0]:.4f}")
    else:
        print("Residuals              : N/A")

    plt.plot(t_m, X @ params, label="Predicted F1 Force")
    plt.plot(t_m, F1, label="Actual Force")
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.title("Couguv Force Estimation")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: sysid_ls.py <data_file.npz> --noisy")
        sys.exit(1)
    solve_ls(sys.argv[1], noisy="--noisy" in sys.argv)

