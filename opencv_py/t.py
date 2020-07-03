import cmath, math, numpy

H_eq = np.dot(H_sample[0:2, :], H_sample[0:2, ].conj().T)
H_eq[0, :] = H_eq[0, :] / np.linalg.norm(H_sample[0, :])
H_eq[1, :] = H_eq[1, :] / np.linalg.norm(H_sample[1, :])
W = np.dot(np.linalg.inv(np.dot(H_eq.conj().T, H_eq) + N0[k] * np.eye(2)), H_eq.conj().T)
WH = np.dot(W, H_eq)
WW = np.dot(W, W.conj().T)
singular_values_out[k, idxBundle, 2] = abs(WH[0, 0]) ** 2 / (abs(WH[0, 1]) ** 2 + N0[k] * abs(WW[0, 0])) / 2
singular_values_out[k, idxBundle, 3] = abs(WH[1, 1]) ** 2 / (abs(WH[1, 0]) ** 2 + N0[k] * abs(WW[1, 1])) / 2