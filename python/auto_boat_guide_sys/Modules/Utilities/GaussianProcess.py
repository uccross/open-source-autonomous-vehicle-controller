"""
.. module:: GaussianProcess.py
    :platform: MacOS, Unix, Windows,
    :synopsis:
.. moduleauthor:: mostly from
https://jessicastringham.net/2018/05/18/Gaussian-Processes/, some modifications
by Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np


class OneDimensional():
    def __init__(self, sigma_f=1.0, ell=1.0, sigma_w=1.0):

        # Vertical scale
        self.sigma_f = sigma_f

        # Horizontal scale
        self.ell = ell

        # Noise
        self.sigma_w = sigma_w
        self.noise = np.random.normal(0.0, sigma_w)

    def calc_cov(self, x_i, x_j):
        K = self.calc_kern(x_i, x_j)
        # K = self.calc_kern_rbf_prd(x_i, x_j)

        return self.calc_kern(x_i, x_j) + np.identity(K.shape[0])*(self.sigma_w**2.0)
        # return self.calc_kern_rbf_prd(x_i, x_j) + np.identity(K.shape[0])*(self.sigma_w**2.0)

    def calc_kern(self, x_i, x_j):
        K = np.zeros((x_i.shape[0], x_j.shape[0]))
        for i in range(x_i.shape[0]):
            for j in range(x_j.shape[0]):
                K[i][j] = (self.sigma_f**2.0)*(
                    np.exp(-(1 / (2*(self.ell**2))) * ((x_i[i] - x_j[j])**2.0)))

        return K

    def calc_kern_rbf_prd(self, x_i, x_j, p=10):
        K = np.zeros((x_i.shape[0], x_j.shape[0]))
        for i in range(x_i.shape[0]):
            for j in range(x_j.shape[0]):
                K[i][j] = ((self.sigma_f**2.0)*(
                    np.exp(-(1 / (2*(self.ell**2))) * ((x_i[i] - x_j[j])**2.0)))
                    *(self.sigma_f**2.0)*np.exp(-
                        (2.0*np.sin(np.pi*np.abs(x_i[i]-x_j[j])/p)**2)/self.ell**2))

        return K


class TwoDimensional():
    def __init__(self, sigma_f=1.0, ell=3.0, sigma_w=1.0, m_field=10,
                 n_field=15, rlvl=2, type="GPR"):
        self.sigma_f = sigma_f
        self.sigma_w = sigma_w
        self.ell = ell
        self.observed_zs = np.zeros((1, 1))
        self.observed_zs_count = 0
        self.observed_yxs = np.zeros((1, 2))
        self.observed_yxs_count = 0
        self.unobserved_yxs = np.zeros((1, 2))
        self.m = m_field
        self.n = n_field
        self.mu = np.zeros((m_field*n_field, 1))
        self.hi = np.zeros((m_field*n_field, 1))
        self.lo = np.zeros((m_field*n_field, 1))
        self.rlvl = rlvl

        self.type = type
        self.Zhat = np.zeros((self.m, self.n))
        self.V_Zhat = np.zeros((self.m, self.n))

        return

    def find_unobserved_yxs(self):
        self.unobserved_yxs = np.zeros((1, 2))
        k = 0
        for i in range(self.m):
            for j in range(self.n):
                u = [i, j]
                if not (u in self.observed_yxs.tolist()):
                    if k == 0:
                        self.unobserved_yxs[0][0] = i
                        self.unobserved_yxs[0][1] = j
                    else:
                        new_yxs = np.array([[i, j]])
                        self.unobserved_yxs = np.concatenate(
                            (self.unobserved_yxs,
                             new_yxs),
                            axis=0)
                k += 1
        return

    def update_observed_yxs(self, oby=0.0, obx=0.0):
        if (self.observed_yxs_count == 0):
            self.observed_yxs[0][0] = oby
            self.observed_yxs[0][1] = obx

        else:
            new_yxs = np.array([[oby, obx]])
            self.observed_yxs = np.concatenate(
                (self.observed_yxs, new_yxs), axis=0)

        self.observed_yxs_count += 1
        return

    def update_observed_zs(self, obz=0.0):
        if (self.observed_zs_count == 0):
            self.observed_zs[0][0] = obz

        else:
            new_z = np.array([[obz]])
            self.observed_zs = np.concatenate(
                (self.observed_zs, new_z), axis=0)

        self.observed_zs_count += 1
        return

    def predict(self):
        #######################################################################
        # TYPE: Gaussian Process Regression (GPR)
        if self.type == "GPR":
            self.mu, cov, _, _, _ = posterior2D(self.unobserved_yxs,
                                               self.observed_yxs,
                                               self.observed_zs, self.ell,
                                               self.sigma_f, self.sigma_w,
                                               N=len(self.observed_zs))

            # estimated values
            for row, mu_row in zip(self.unobserved_yxs, self.mu):
                i = int(row[0])
                j = int(row[1])

                self.Zhat[i][j] = mu_row[0]

            # filling in with real values
            for row, obz in zip(self.observed_yxs, self.observed_zs):
                i = int(row[0])
                j = int(row[1])

                self.Zhat[i][j] = obz[0]

            # Variance matrix
            # Variance matrix associated with the predicted field
            # self.hi, self.lo = upper_lower_bounds(self.mu, cov)
            self.hi, self.lo, var = upper_lower_bounds_relative(cov)

            # estimated values
            for u_yxs_row, var_row in zip(self.unobserved_yxs, var):
                i = int(u_yxs_row[0])
                j = int(u_yxs_row[1])

                self.V_Zhat[i][j] = var_row[0]

        #######################################################################
        # TYPE: Partitioned Gaussian Process Regression (PGPR)
        elif self.type == "PGPR":

            q_yxs, q_zs, u_yxs = self.isolate_points_in_quad(
                self.observed_yxs, self.observed_zs, end_level=self.rlvl)

            # TODO: find a way to determine and incorporate ell, sigma f and
            # sigma w
            q_mu, q_cov, _, _, _ = posterior2D(u_yxs,
                                              q_yxs,
                                              q_zs, self.ell,
                                              self.sigma_f, self.sigma_w,
                                              N=len(q_zs))

            q_hi, q_lo, var = upper_lower_bounds_relative(q_cov)

            k = 0
            l = 0
            for i in range(self.m):
                for j in range(self.n):
                    u = [i, j]
                    if u in u_yxs.tolist():
                        self.mu[k] = q_mu[l]
                        self.hi[k] = q_hi[l]
                        self.lo[k] = q_lo[l]
                        l += 1
                    k += 1

            # estimated values
            for yx_row, mu_row in zip(u_yxs, q_mu):
                i = int(yx_row[0])
                j = int(yx_row[1])

                self.Zhat[i][j] = mu_row[0]

            # filling in with real measured values
            for row, obz in zip(q_yxs, q_zs):
                i = int(row[0])
                j = int(row[1])

                self.Zhat[i][j] = obz[0]

            # estimated values
            for u_yxs_row, var_row in zip(u_yxs, var):
                i = int(u_yxs_row[0])
                j = int(u_yxs_row[1])

                self.V_Zhat[i][j] = var_row[0]

        return self.Zhat, self.V_Zhat  # , hi, lo#, std_hi, std_lo

    def isolate_points_in_quad(self, observed_yxs, observed_zs, end_level=1):

        latest_yx = observed_yxs[-1][:]

        # Get the quadrant
        [ix0, ixf, iy0, iyf] = self.get_quadrant(latest_yx, end_level)

        # Get the observed yxs and zs in the quadrant
        ob_yxs_in_quad = np.zeros((1, 2))
        ob_zs_in_quad = np.zeros((1, 1))

        i = 0
        for yx, z in zip(observed_yxs, observed_zs):
            if self.is_yx_in_quadrant(yx, ix0, ixf, iy0, iyf):
                if i == 0:
                    ob_yxs_in_quad[0][0] = yx[0]
                    ob_yxs_in_quad[0][1] = yx[1]

                    ob_zs_in_quad[0][0] = z[0]
                else:
                    ob_yxs_in_quad = np.concatenate(
                        (ob_yxs_in_quad, np.array([yx])), axis=0)

                    ob_zs_in_quad = np.concatenate(
                        (ob_zs_in_quad, np.array([z])), axis=0)

                i += 1

        # Get the unobserved yxs and zs in the quadrant
        unob_yxs_in_quad = np.zeros((1, 2))
        # unob_zs_in_quad = np.zeros((1, 1))

        k = 0
        yrng = range(iy0, iyf)
        xrng = range(ix0, ixf)
        for i in yrng:
            for j in xrng:
                u = [i, j]
                if not (u in ob_yxs_in_quad.tolist()):
                    if k == 0:
                        unob_yxs_in_quad[0][0] = i
                        unob_yxs_in_quad[0][1] = j
                    else:
                        unob_yxs_in_quad = np.concatenate(
                            (unob_yxs_in_quad, np.array([[i, j]])), axis=0)
                k+=1 

        return ob_yxs_in_quad, ob_zs_in_quad, unob_yxs_in_quad#, yrng, xrng

    def get_quadrant(self, yx, end_level=1):

        # print("yx.shape: {}".format(yx.shape))

        # Binary search in x for minimum x partition coordinate
        qx0, qxf = self.binary_search(yx[1], 0, self.n, 0, end_level)

        # Binary search in y for minimum y partition coordinate
        qy0, qyf = self.binary_search(yx[0], 0, self.m, 0, end_level)

        return [qx0, qxf, qy0, qyf]

    def binary_search(self, iv, il, ir, cur_level, end_level):
        """
        binary search
        :param iv: index value to be searched with
        :param il: left index
        :param ir: right index
        :param cur_level: the current level of recursion
        :param end_level: the recursion-end level to stop recurring at (0 is
        the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
        levels)
        :return: divs[il], divs[ir]
        """

        # Termination, or "Bottom-out" case
        if cur_level >= end_level:
            return [il, ir]

        i = round((il+ir)/2)

        # bias towards left
        if iv <= i:
            ir = i

        else:
            il = i

        return self.binary_search(iv, il, ir, cur_level+1, end_level)

    def is_yx_in_quadrant(self, yx, ix0, ixf, iy0, iyf):
        """
        :param position: np.array([[east , north]]) a position index vector 1x2
        :param ix0:
        :param ixf:
        :param iy0:
        :param iyf:
        """

        if ((ix0 <= yx[1]) and (yx[1] <= ixf) and
                (iy0 <= yx[0]) and (yx[0] <= iyf)):
            return True

        return False

    def get_observed_yxs(self):
        return self.observed_yxs

    def get_unobserved_yxs(self):
        return self.unobserved_yxs

    def get_mu(self):
        return self.mu

    def get_hi(self):
        return self.hi

    def get_lo(self):
        return self.lo

    def set_ell(self, ell_new):
        self.ell = ell_new

    def set_sigma_f(self, sigma_new):
        self.sigma_f = sigma_new

    def set_rlvl(self, lvl):
        self.rlvl = lvl

###############################################################################
# Helper Functions
def split_up_xgrid(xs, N):
    '''Choose N random points from raw_xs, return those N points and
    the remaining points
    '''
    # Thanks https://stackoverflow.com/a/32726407
    point_i = np.full(xs.shape, False, bool)
    a = np.random.choice(point_i.shape[0], N, replace=False)
    point_i[a] = True
    return xs[point_i], xs[~point_i]


def upper_lower_bounds(mu, cov):
    '''returns the lower and upper bounds based on stdevs

    mu: shape (x,)
    cov: shape (x, x)
    '''
    # compute error bars based on variance.
    stds = np.sqrt(np.diagonal(cov))

    # forcing everything to be the right size
    return (mu[:, None] - 2 * stds[:, None])[:, 0], (mu[:, None] + 2 * stds[:, None])[:, 0]


def upper_lower_bounds_relative(cov):
    '''returns the lower and upper bounds based on stdevs

    cov: shape (x, x)
    '''
    # compute error bars based on variance.
    stds = np.sqrt(np.diagonal(cov))

    var = np.diagonal(cov)

    # forcing everything to be the right size
    return (-2*stds[:, None])[:, 0], (2 * stds[:, None])[:, 0], var[:, None]


# I'm dealing with a single input, so I'll explicitly write out x and b
def sig(x, w, b): return 1 / (1 + np.exp(-(w * x + b)))

# This is what we've been waiting for!


def f(x):
    # # don't mind me, this is just to help build f
    # f_params = [
    #     (5, 0.5, -1),
    #     (-2, 5, 1),
    #     (-4, 3, 10),
    # ]
    # return np.sum(
    #     ((c * sig(x, w, b)) for (c, w, b) in f_params)
    # )

    return np.cos(x)*np.exp(-x*0.1)


def slct_rand_pnts_frm_i(x_indices, N):
    """
    :param x_indices: A 'n' by 1 vector of indices
    :param N: N points to generate from the indices
    """
    return np.random.choice(x_indices.shape[0], N, replace=True)


def k2D(yxi, yxj, ell_1, ell_2, sigma_f):
    yi = yxi[0]
    xi = yxi[1]

    yj = yxj[0]
    xj = yxj[1]

    return sigma_f**2 * np.exp((-((xi - xj)/ell_1)**2/2) +
                               (-((yi - yj)/ell_2)**2/2))


def calc_kern_2D(yx_i, yx_j, ell, sigma_f):
    K = np.zeros((yx_i.shape[0], yx_j.shape[0]))
    for i in range(yx_i.shape[0]):
        for j in range(yx_j.shape[0]):
            # K[i][j] = k2D(yx_i[i], yx_j[j], ell, ell, sigma_f)
            K[i][j] = k2D(yx_i[i], yx_j[j], ell, ell, sigma_f)
    return K


def posterior2D(unobserved_yxs, observed_yxs, observed_zs, ell, sigma_f,
               sigma_w, N):
    A = calc_kern_2D(unobserved_yxs, unobserved_yxs, ell, sigma_f)

    B = calc_kern_2D(observed_yxs, observed_yxs, ell, sigma_f) + \
        (sigma_w**2 * np.identity(N))

    C = calc_kern_2D(unobserved_yxs, observed_yxs, ell, sigma_f)

    mu = C @ np.linalg.solve(B, observed_zs)

    cov = A - C @ np.linalg.solve(B, C.T)

    return mu, cov, A, B, C


###########################################################################
# Unit test(s)
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib import cm
    print("Testing GaussianProcess.py 1D")

    sigma_f = 0.5
    ell = 1.0
    sigma_w = 0.1

    gp = OneDimensional(sigma_f=sigma_f, ell=ell, sigma_w=sigma_w)

    x0 = -10
    xf = 10
    n = 1000

    x = np.linspace(x0, xf, n)

    m = 20

    x_observed, x_unobserved = split_up_xgrid(x, m)

    observed_ys = f(x_observed) + (sigma_w *
                                   np.random.randn(x_observed.shape[0]))

    # x_observed = x_nth[0:m]
    # x_unobserved = x_nth[m:]
    print("x_observed: {}".format(x_observed))
    print("x_unobserved: {}".format(x_unobserved))
    print("x_observed.shape: {}".format(x_observed.shape))
    print("x_unobserved.shape: {}".format(x_unobserved.shape))

    ###########################################################################
    # posterior
    A = gp.calc_kern(x_unobserved, x_unobserved)
    B = gp.calc_cov(x_observed, x_observed)
    C = gp.calc_kern(x_unobserved, x_observed)

    ###########################################################################
    fig = plt.figure()
    ax = plt.gca()
    points_cov = ax.matshow(A,
                            cmap=cm.plasma,
                            vmin=np.min(A),
                            vmax=np.max(A),
                            origin='lower')
    fig.colorbar(points_cov, shrink=0.5, ax=ax, location='right')
    ax.set_title('A')
    plt.show

    fig = plt.figure()
    ax = plt.gca()
    points_cov = ax.matshow(B,
                            cmap=cm.plasma,
                            vmin=np.min(B),
                            vmax=np.max(B),
                            origin='lower')
    fig.colorbar(points_cov, shrink=0.5, ax=ax, location='right')
    ax.set_title('B')
    plt.show

    fig = plt.figure()
    ax = plt.gca()
    points_cov = ax.matshow(C,
                            cmap=cm.plasma,
                            vmin=np.min(C),
                            vmax=np.max(C),
                            origin='lower')
    fig.colorbar(points_cov, shrink=0.5, ax=ax, location='right')
    ax.set_title('C')
    plt.show

    ###########################################################################

    print("observed_ys.shape: {}".format(observed_ys.shape))
    print("A.shape: {}".format(A.shape))
    print("B.shape: {}".format(B.shape))
    print("C.shape: {}".format(C.shape))

    mu = C @ np.linalg.solve(B, observed_ys)
    cov = A - C @ np.linalg.solve(B, C.T)

    # compute error bars based on variance.
    stds = np.sqrt(np.diagonal(cov))

    error_low, error_high = upper_lower_bounds(mu, cov)

    # error_low = (mu[:, None] - 2 * stds[:, None])[:, 0]
    # error_high = (mu[:, None] + 2 * stds[:, None])[:, 0]
    error_low = np.reshape(error_low, (len(x_unobserved),))
    error_high = np.reshape(error_high, (len(x_unobserved),))
    print("error_low.shape: {}".format(error_low.shape))
    print("error_high.shape: {}".format(error_high.shape))

    # Covariance matrix
    fig = plt.figure()
    ax = plt.gca()
    points_cov = ax.matshow(cov,
                            cmap=cm.plasma,
                            vmin=np.min(cov),
                            vmax=np.max(cov),
                            origin='lower')
    fig.colorbar(points_cov, shrink=0.5, ax=ax, location='right')
    ax.set_title('cov')
    plt.show

    fig = plt.figure()
    ax = plt.gca()
    ax.plot(x, f(x), label='truth', color='blue')
    ax.plot(x_unobserved, mu, label='mean', color='red')
    ax.fill_between(x_unobserved, error_low, error_high, label='error',
                    color='lightsteelblue')
    ax.scatter(x_observed, observed_ys, label='measurement')
    ax.set_xlabel('x')
    ax.set_ylabel('f(x)')
    ax.set_title('1-Dimensional GPR Example')
    ax.grid()
    ax.legend()
    plt.show()
