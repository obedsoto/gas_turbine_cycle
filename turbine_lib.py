from gases import *
import gas_dynamics as gd
from network_lib import *
import functions as func


class Compressor(Unit):
    def __init__(self, pi_c, work_fluid=Air(), eta_stag_p=0.89, precision=0.01):
        self.gd_inlet_port = GasDynamicPort()
        self.gd_outlet_port = GasDynamicPort()
        self.m_inlet_port = MechanicalPort()
        self.eta_stag_p = eta_stag_p
        self.pi_c = pi_c
        self.work_fluid = work_fluid
        self.precision = precision
        self._k = self.work_fluid.k_av_int
        self._k_res = 1
        self._k_old = None
        self._eta_stag = None
        self._L = None

    @property
    def k_old(self):
        return self._k_old

    @property
    def k(self):
        return self._k

    @property
    def eta_stag(self):
        return self._eta_stag

    @property
    def L(self):
        return self._L

    def _check(self):
        return self.gd_inlet_port.linked_connection.check() and self.pi_c is not None and self.eta_stag_p is not None

    @property
    def T_stag_in(self):
        return self.gd_inlet_port.linked_connection.T_stag

    @T_stag_in.setter
    def T_stag_in(self, value):
        self.gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_in(self):
        return self.gd_inlet_port.linked_connection.p_stag

    @p_stag_in.setter
    def p_stag_in(self, value):
        self.gd_inlet_port.linked_connection.p_stag = value

    @property
    def g_in(self):
        return self.gd_inlet_port.linked_connection.g

    @g_in.setter
    def g_in(self, value):
        self.gd_inlet_port.linked_connection.g = value

    @property
    def T_stag_out(self):
        return self.gd_outlet_port.linked_connection.T_stag

    @T_stag_out.setter
    def T_stag_out(self, value):
        self.gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_out(self):
        return self.gd_outlet_port.linked_connection.p_stag

    @p_stag_out.setter
    def p_stag_out(self, value):
        self.gd_outlet_port.linked_connection.p_stag = value

    @property
    def g_out(self):
        return self.gd_outlet_port.linked_connection.g

    @g_out.setter
    def g_out(self, value):
        self.gd_outlet_port.linked_connection.g = value

    def update_connection_current_state(self, relax_coef=1):
        if self._check():
            self.gd_outlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of gd_outlet_port connection:')
            self.gd_outlet_port.linked_connection.log_state()
            self.m_inlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of m_inlet_port connection:')
            self.m_inlet_port.linked_connection.log_state()

    def update(self, relax_coef=1):
        if self._check():
            self.work_fluid.__init__()
            self.work_fluid.T1 = self.T_stag_in
            while self._k_res >= self.precision:
                self._eta_stag = func.eta_comp_stag(self.pi_c, self._k, self.eta_stag_p)
                self.work_fluid.T2 = self.T_stag_in * (1 + (self.pi_c ** ((self._k - 1) / self._k) - 1) /
                                                       self._eta_stag)
                self.T_stag_out = self.work_fluid.T2
                self._k_old = self._k
                self._k = self.work_fluid.k_av_int
                self._k_res = abs(self._k - self._k_old) / self._k_old
            self._L = self.work_fluid.c_p_av_int * (self.T_stag_out - self.T_stag_in)
            self.p_stag_out = self.p_stag_in * self.pi_c
            self.g_out = 1
            self.m_inlet_port.linked_connection.L_outlet = self._L
            self.gd_outlet_port.linked_connection.alpha = self.gd_inlet_port.linked_connection.alpha
            self.gd_outlet_port.linked_connection.g_fuel = self.gd_inlet_port.linked_connection.g_fuel


class Turbine(Unit):
    def __init__(self, work_fluid=KeroseneCombustionProducts(), eta_stag_p=0.91, precision=0.01):
        self.gd_inlet_port = GasDynamicPort()
        self.gd_outlet_port = GasDynamicPort()
        self.m_comp_outlet_port = MechanicalPort()
        self.m_load_outlet_port = MechanicalPort()
        self.eta_stag_p = eta_stag_p
        self.precision = precision
        self.work_fluid = work_fluid
        self._k = self.work_fluid.k_av_int
        self._k_old = None
        self._k_res = None
        self._pi_t = None
        self._eta_stag = None
        self._L = None
        self._pi_t = None
        self._pi_t_res = 1
        self._pi_t_old = None

    @property
    def k(self):
        return self._k

    @property
    def k_old(self):
        return self._k_old

    @property
    def L(self):
        return self._L

    @property
    def pi_t(self):
        return self._pi_t

    @property
    def eta_stag(self):
        return self._eta_stag

    @property
    def T_stag_in(self):
        return self.gd_inlet_port.linked_connection.T_stag

    @T_stag_in.setter
    def T_stag_in(self, value):
        self.gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_in(self):
        return self.gd_inlet_port.linked_connection.p_stag

    @p_stag_in.setter
    def p_stag_in(self, value):
        self.gd_inlet_port.linked_connection.p_stag = value

    @property
    def g_in(self):
        return self.gd_inlet_port.linked_connection.g

    @g_in.setter
    def g_in(self, value):
        self.gd_inlet_port.linked_connection.g = value

    @property
    def T_stag_out(self):
        return self.gd_outlet_port.linked_connection.T_stag

    @T_stag_out.setter
    def T_stag_out(self, value):
        self.gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_out(self):
        return self.gd_outlet_port.linked_connection.p_stag

    @p_stag_out.setter
    def p_stag_out(self, value):
        self.gd_outlet_port.linked_connection.p_stag = value

    @property
    def g_out(self):
        return self.gd_outlet_port.linked_connection.g

    @g_out.setter
    def g_out(self, value):
        self.gd_outlet_port.linked_connection.g = value

    @property
    def alpha(self):
        return self.gd_inlet_port.linked_connection.alpha

    @alpha.setter
    def alpha(self, value):
        self.gd_inlet_port.linked_connection.alpha = value

    def _check(self):
        return self.gd_inlet_port.linked_connection.check() \
               and self.eta_stag_p is not None \
               and self.alpha is not None \
               and self.alpha != np.inf

    def _check_power_turbine(self):
        return self._check() \
               and self.m_comp_outlet_port.linked_connection.L_inlet is not None \
               and self.gd_outlet_port.linked_connection.p_stag is not None and \
               self.m_load_outlet_port.linked_connection.L_inlet != 0

    def _check_comp_turbine_p_in(self):
        return self._check() \
               and self.m_comp_outlet_port.linked_connection.L_inlet is not None \
               and self.m_load_outlet_port.linked_connection.L_inlet == 0

    def _check_comp_turbine_p_out(self):
        return self.gd_outlet_port.linked_connection.p_stag is not None \
               and self.gd_inlet_port.linked_connection.T_stag is not None \
               and self.gd_inlet_port.linked_connection.g is not None \
               and self.m_comp_outlet_port.linked_connection.L_inlet is not None \
               and self.m_load_outlet_port.linked_connection.L_inlet == 0

    def update_connection_current_state(self, relax_coef=1):
        if self._check_power_turbine():
            self.m_load_outlet_port.linked_connection.update_current_state(relax_coef)
            self.gd_outlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of m_load_port connection:')
            self.m_load_outlet_port.linked_connection.log_state()
            logging.debug('New state of gd_outlet_port connection')
            self.gd_outlet_port.linked_connection.log_state()
        elif self._check_comp_turbine_p_in():
            self.gd_outlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of gd_outlet_port connection')
            self.gd_outlet_port.linked_connection.log_state()
        elif self._check_comp_turbine_p_out():
            self.gd_outlet_port.linked_connection.update_current_state(relax_coef)
            self.gd_inlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of gd_outlet_port connection')
            self.gd_outlet_port.linked_connection.log_state()
            logging.debug('New state of gd_inlet_port connection')
            self.gd_inlet_port.linked_connection.log_state()

    def _compute_compressor_turbine(self):
        self._k_res = 1
        self._pi_t_res = 1
        self.work_fluid.__init__()
        self.work_fluid.alpha = self.alpha
        self.work_fluid.T1 = self.T_stag_in
        self.g_out = self.g_in
        self.gd_outlet_port.linked_connection.g_fuel = self.gd_inlet_port.linked_connection.g_fuel
        self._L = self.m_comp_outlet_port.linked_connection.L_inlet / self.g_in
        while self._k_res >= self.precision:
            self.T_stag_out = self.T_stag_in - self._L / self.work_fluid.c_p_av_int
            self.work_fluid.T2 = self.T_stag_out
            self._k_old = self._k
            self._k = self.work_fluid.k_av_int
            self._k_res = abs(self._k - self._k_old) / self._k_old
        self._pi_t = (1 - self._L / (self.T_stag_in * self.work_fluid.c_p_av_int * self.eta_stag_p)) ** \
                     (self._k / (1 - self._k))
        while self._pi_t_res >= self.precision:
            self._eta_stag = func.eta_turb_stag(self._pi_t, self._k, self.eta_stag_p)
            self._pi_t_old = self._pi_t
            self._pi_t = (1 - self._L / (self.T_stag_in * self.work_fluid.c_p_av_int * self._eta_stag)) ** \
                         (self._k / (1 - self._k))
            self._pi_t_res = abs(self._pi_t - self._pi_t_old) / self._pi_t_old

    def update(self):
        self.gd_outlet_port.linked_connection.alpha = self.alpha
        if self._check_power_turbine():
            self._k_res = 1
            self.work_fluid.__init__()
            self.work_fluid.alpha = self.alpha
            self.work_fluid.T1 = self.T_stag_in
            self._pi_t = self.p_stag_in / self.p_stag_out
            while self._k_res >= self.precision:
                self._eta_stag = func.eta_turb_stag(self._pi_t, self._k, self.eta_stag_p)
                self.work_fluid.T2 = self.T_stag_in * (1 - (1 - self._pi_t **
                                                            ((1 - self._k) / self._k)) * self._eta_stag)
                self.T_stag_out = self.work_fluid.T2
                self._k_old = self._k
                self._k = self.work_fluid.k_av_int
                self._k_res = abs(self._k - self._k_old) / self._k_old
            self._L = self.work_fluid.c_p_av_int * (self.T_stag_in - self.T_stag_out)
            self.g_out = self.g_in
            self.gd_outlet_port.linked_connection.g_fuel = self.gd_inlet_port.linked_connection.g_fuel
            self.m_load_outlet_port.linked_connection.L_inlet = self._L * self.g_in - \
                                                                self.m_comp_outlet_port.linked_connection.L_inlet

        elif self._check_comp_turbine_p_in():
            self._compute_compressor_turbine()
            self.p_stag_out = self.p_stag_in / self._pi_t

        elif self._check_comp_turbine_p_out():
            self._compute_compressor_turbine()
            self.p_stag_in = self.p_stag_out * self._pi_t


class CombustionChamber(Unit):
    def __init__(self, Q_n, l0, precision=0.01, eta_burn=0.98, sigma_comb=0.98, g_outflow=0.01,
                 g_cooling=0.04, g_return=0.02, work_fluid_in=Air(), work_fluid_out=KeroseneCombustionProducts()):
        self.gd_inlet_port = GasDynamicPort()
        self.gd_outlet_port = GasDynamicPort()
        self.work_fluid_in = work_fluid_in
        self.work_fluid_out = work_fluid_out
        self.work_fluid_out_T0 = self.work_fluid_out
        self.precision = precision
        self.eta_burn = eta_burn
        self.Q_n = Q_n
        self.l0 = l0
        self.sigma_comb = sigma_comb
        self.g_outflow = g_outflow
        self.g_cooling = g_cooling
        self.g_return = g_return
        self._alpha_res = 1
        self._alpha_out_old = None
        self._g_fuel_prime = 0

    @property
    def T_stag_in(self):
        return self.gd_inlet_port.linked_connection.T_stag

    @T_stag_in.setter
    def T_stag_in(self, value):
        self.gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_in(self):
        return self.gd_inlet_port.linked_connection.p_stag

    @p_stag_in.setter
    def p_stag_in(self, value):
        self.gd_inlet_port.linked_connection.p_stag = value

    @property
    def g_in(self):
        return self.gd_inlet_port.linked_connection.g

    @g_in.setter
    def g_in(self, value):
        self.gd_inlet_port.linked_connection.g = value

    @property
    def g_fuel_in(self):
        return self.gd_inlet_port.linked_connection.g_fuel

    @g_fuel_in.setter
    def g_fuel_in(self, value):
        self.gd_inlet_port.linked_connection.g_fuel = value

    @property
    def T_stag_out(self):
        return self.gd_outlet_port.linked_connection.T_stag

    @T_stag_out.setter
    def T_stag_out(self, value):
        self.gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_out(self):
        return self.gd_outlet_port.linked_connection.p_stag

    @p_stag_out.setter
    def p_stag_out(self, value):
        self.gd_outlet_port.linked_connection.p_stag = value

    @property
    def g_out(self):
        return self.gd_outlet_port.linked_connection.g

    @g_out.setter
    def g_out(self, value):
        self.gd_outlet_port.linked_connection.g = value

    @property
    def alpha_out(self):
        return self.gd_outlet_port.linked_connection.alpha

    @alpha_out.setter
    def alpha_out(self, value):
        self.gd_outlet_port.linked_connection.alpha = value

    @property
    def g_fuel_out(self):
        return self.gd_outlet_port.linked_connection.g_fuel

    @g_fuel_out.setter
    def g_fuel_out(self, value):
        self.gd_outlet_port.linked_connection.g_fuel = value

    def _check(self):
        return self.gd_inlet_port.linked_connection.check() and \
               self.alpha_out is not None and \
               self.eta_burn is not None and \
               self.Q_n is not None and \
               self.l0 is not None and \
               self.sigma_comb is not None and \
               self.T_stag_out is not None and \
               self.g_cooling is not None and \
               self.g_outflow is not None and \
               self.g_return is not None and \
               self.g_fuel_in is not None

    def update_connection_current_state(self, relax_coef=1):
        if self._check():
            self.gd_outlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of gd_outlet_port connection')
            self.gd_outlet_port.linked_connection.log_state()

    def update(self):
        if self._check():
            self._alpha_res = 1
            self.work_fluid_in.__init__()
            self.work_fluid_out.__init__()
            self.work_fluid_out_T0.__init__()
            self.work_fluid_out.alpha = self.alpha_out
            self.work_fluid_out_T0.alpha = self.alpha_out
            self.work_fluid_out.T = self.T_stag_out
            self.work_fluid_out_T0.T = 288
            self.work_fluid_in.T = self.T_stag_in
            while self._alpha_res >= self.precision:
                self._g_fuel_prime = (self.work_fluid_out.c_p_av * self.T_stag_out -
                                      self.work_fluid_in.c_p_av * self.T_stag_in) / \
                                     (self.Q_n * self.eta_burn - self.work_fluid_out.c_p_av * self.T_stag_out +
                                      self.work_fluid_out_T0.c_p_av * 288)
                self.g_out = (1 + self.g_fuel_in + self._g_fuel_prime) * (1 - self.g_cooling - self.g_outflow) + \
                              self.g_return
                self._alpha_out_old = self.work_fluid_out.alpha
                self.alpha_out = 1 / (self.l0 * (self.g_fuel_in + self._g_fuel_prime))
                self.g_fuel_out = self.g_fuel_in + self._g_fuel_prime
                self.work_fluid_out.alpha = self.alpha_out
                self.work_fluid_out_T0.alpha = self.alpha_out
                self._alpha_res = abs(self._alpha_out_old - self.alpha_out) / self.alpha_out
            self.p_stag_out = self.p_stag_in * self.sigma_comb


class Inlet(Unit):
    def __init__(self, sigma=0.99, work_fluid=Air()):
        self.sigma = sigma
        self.gd_inlet_port = GasDynamicPort()
        self.gd_outlet_port = GasDynamicPort()
        self.work_fluid = work_fluid

    @property
    def T_stag_in(self):
        return self.gd_inlet_port.linked_connection.T_stag

    @T_stag_in.setter
    def T_stag_in(self, value):
        self.gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_in(self):
        return self.gd_inlet_port.linked_connection.p_stag

    @p_stag_in.setter
    def p_stag_in(self, value):
        self.gd_inlet_port.linked_connection.p_stag = value

    @property
    def g_in(self):
        return self.gd_inlet_port.linked_connection.g

    @g_in.setter
    def g_in(self, value):
        self.gd_inlet_port.linked_connection.g = value

    @property
    def T_stag_out(self):
        return self.gd_outlet_port.linked_connection.T_stag

    @T_stag_out.setter
    def T_stag_out(self, value):
        self.gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_out(self):
        return self.gd_outlet_port.linked_connection.p_stag

    @p_stag_out.setter
    def p_stag_out(self, value):
        self.gd_outlet_port.linked_connection.p_stag = value

    @property
    def g_out(self):
        return self.gd_outlet_port.linked_connection.g

    @g_out.setter
    def g_out(self, value):
        self.gd_outlet_port.linked_connection.g = value

    def _check(self):
        return self.p_stag_in is not None and self.T_stag_in is not None and self.g_in is not None \
               and self.sigma is not None

    def update_connection_current_state(self, relax_coef=1):
        if self._check():
            self.gd_outlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of gd_outlet_port connection')
            self.gd_outlet_port.linked_connection.log_state()

    def update(self):
        if self._check():
            self.p_stag_out = self.p_stag_in * self.sigma
            self.T_stag_out = self.T_stag_in
            self.g_out = self.g_in
            self.gd_outlet_port.linked_connection.alpha = self.gd_inlet_port.linked_connection.alpha
            self.gd_outlet_port.linked_connection.g_fuel = self.gd_inlet_port.linked_connection.g_fuel


class Outlet(Unit):
    def __init__(self, sigma=0.99, work_fluid=KeroseneCombustionProducts()):
        self.gd_inlet_port = GasDynamicPort()
        self.gd_outlet_port = GasDynamicPort()
        self.sigma = sigma
        self.work_fluid = work_fluid

    @property
    def T_stag_in(self):
        return self.gd_inlet_port.linked_connection.T_stag

    @T_stag_in.setter
    def T_stag_in(self, value):
        self.gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_in(self):
        return self.gd_inlet_port.linked_connection.p_stag

    @p_stag_in.setter
    def p_stag_in(self, value):
        self.gd_inlet_port.linked_connection.p_stag = value

    @property
    def g_in(self):
        return self.gd_inlet_port.linked_connection.g

    @g_in.setter
    def g_in(self, value):
        self.gd_inlet_port.linked_connection.g = value

    @property
    def T_stag_out(self):
        return self.gd_outlet_port.linked_connection.T_stag

    @T_stag_out.setter
    def T_stag_out(self, value):
        self.gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_out(self):
        return self.gd_outlet_port.linked_connection.p_stag

    @p_stag_out.setter
    def p_stag_out(self, value):
        self.gd_outlet_port.linked_connection.p_stag = value

    @property
    def g_out(self):
        return self.gd_outlet_port.linked_connection.g

    @g_out.setter
    def g_out(self, value):
        self.gd_outlet_port.linked_connection.g = value

    @property
    def alpha(self):
        return self.gd_inlet_port.linked_connection.alpha

    @alpha.setter
    def alpha(self, value):
        self.gd_inlet_port.linked_connection.alpha = value

    def _check1(self):
        return self.sigma is not None and self.p_stag_out

    def _check2(self):
        return self.T_stag_in is not None

    def update_connection_current_state(self, relax_coef=1):
        if self._check1() and self._check2():
            self.gd_inlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of gd_outlet_port connection')
            self.gd_outlet_port.linked_connection.log_state()
            logging.debug('New state of gd_inlet_port connection')
            self.gd_inlet_port.linked_connection.log_state()

    def update(self):
        if self._check2():
            self.T_stag_out = self.T_stag_in
            self.gd_outlet_port.linked_connection.alpha = self.alpha
            self.g_out = self.g_in
        if self._check1() and self._check2():
            self.T_stag_out = self.T_stag_in
            self.gd_outlet_port.linked_connection.alpha = self.alpha
            self.g_out = self.g_in
            self.p_stag_in = self.p_stag_out / self.sigma
            self.gd_outlet_port.linked_connection.g_fuel = self.gd_inlet_port.linked_connection.g_fuel


class Atmosphere(Unit):
    def __init__(self, p0=1e5, T0=288, lam_in=0.04, work_fluid_in=KeroseneCombustionProducts(), work_fluid_out=Air()):
        self.p0 = p0
        self.T0 = T0
        self.lam_in = lam_in
        self.work_fluid_in = work_fluid_in
        self.work_fluid_out = work_fluid_out
        self.gd_inlet_port = GasDynamicPort()
        self.gd_outlet_port = GasDynamicPort()

    @property
    def T_stag_in(self):
        return self.gd_inlet_port.linked_connection.T_stag

    @T_stag_in.setter
    def T_stag_in(self, value):
        self.gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_in(self):
        return self.gd_inlet_port.linked_connection.p_stag

    @p_stag_in.setter
    def p_stag_in(self, value):
        self.gd_inlet_port.linked_connection.p_stag = value

    @property
    def T_stag_out(self):
        return self.gd_outlet_port.linked_connection.T_stag

    @T_stag_out.setter
    def T_stag_out(self, value):
        self.gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_out(self):
        return self.gd_outlet_port.linked_connection.p_stag

    @p_stag_out.setter
    def p_stag_out(self, value):
        self.gd_outlet_port.linked_connection.p_stag = value

    def _check1(self):
        return self.lam_in is not None and self.p0 is not None and self.T0 is not None

    def _check2(self):
        return self.T_stag_in is not None

    def update_connection_current_state(self, relax_coef=1):
        if self._check1() and self._check2():
            self.gd_inlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of gd_inlet_port connection')
            self.gd_inlet_port.linked_connection.log_state()

    def update(self):
        if self._check1():
            self.T_stag_out = self.T0
            self.p_stag_out = self.p0
        if self._check1() and self._check2():
            self.T_stag_out = self.T0
            self.p_stag_out = self.p0
            self.work_fluid_in.__init__()
            self.work_fluid_out.__init__()
            self.work_fluid_in.T = self.T_stag_in
            self.p_stag_in = self.p0 / gd.GasDynamicFunctions.pi_lam(self.lam_in, self.work_fluid_in.k)


class Load(Unit):
    def __init__(self, power: float =0):
        self.power = power
        self.m_inlet_port = MechanicalPort()
        self._G_air = None

    @property
    def G_air(self):
        return self._G_air

    @property
    def L(self):
        return self.m_inlet_port.linked_connection.L_outlet

    @L.setter
    def L(self, value):
        self.m_inlet_port.linked_connection.L_outlet = value

    def update_connection_current_state(self, relax_coef=1):
        logging.debug('New state of m_inlet_port connection')
        self.m_inlet_port.linked_connection.log_state()

    def update(self):
        if self.power != 0 and self.L is not None and self.L != 0:
            self._G_air = self.power / self.L
        elif self.power == 0:
            self.L = 0


class Regenerator(Unit):
    def __init__(self, regeneration_rate, sigma_hot=0.99, sigma_cold=0.99, work_fluid_cold=Air(),
                 work_fluid_hot=KeroseneCombustionProducts()):
        self.regeneration_rate = regeneration_rate
        self.sigma_hot = sigma_hot
        self.sigma_cold = sigma_cold
        self.hot_gd_inlet_port = GasDynamicPort()
        self.hot_gd_outlet_port = GasDynamicPort()
        self.cold_gd_inlet_port = GasDynamicPort()
        self.cold_gd_outlet_port = GasDynamicPort()
        self.work_fluid_hot = work_fluid_hot
        self.work_fluid_cold = work_fluid_cold

    @property
    def T_stag_hot_in(self):
        return self.hot_gd_inlet_port.linked_connection.T_stag

    @T_stag_hot_in.setter
    def T_stag_hot_in(self, value):
        self.hot_gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_hot_in(self):
        return self.hot_gd_inlet_port.linked_connection.p_stag

    @p_stag_hot_in.setter
    def p_stag_hot_in(self, value):
        self.hot_gd_inlet_port.linked_connection.p_stag = value

    @property
    def T_stag_hot_out(self):
        return self.hot_gd_outlet_port.linked_connection.T_stag

    @T_stag_hot_out.setter
    def T_stag_hot_out(self, value):
        self.hot_gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_hot_out(self):
        return self.hot_gd_outlet_port.linked_connection.p_stag

    @p_stag_hot_out.setter
    def p_stag_hot_out(self, value):
        self.hot_gd_outlet_port.linked_connection.p_stag = value

    @property
    def T_stag_cold_in(self):
        return self.cold_gd_inlet_port.linked_connection.T_stag

    @T_stag_cold_in.setter
    def T_stag_cold_in(self, value):
        self.cold_gd_inlet_port.linked_connection.T_stag = value

    @property
    def p_stag_cold_in(self):
        return self.cold_gd_inlet_port.linked_connection.p_stag

    @p_stag_cold_in.setter
    def p_stag_cold_in(self, value):
        self.cold_gd_inlet_port.linked_connection.p_stag = value

    @property
    def T_stag_cold_out(self):
        return self.cold_gd_outlet_port.linked_connection.T_stag

    @T_stag_cold_out.setter
    def T_stag_cold_out(self, value):
        self.cold_gd_outlet_port.linked_connection.T_stag = value

    @property
    def p_stag_cold_out(self):
        return self.cold_gd_outlet_port.linked_connection.p_stag

    @p_stag_cold_out.setter
    def p_stag_cold_out(self, value):
        self.cold_gd_outlet_port.linked_connection.p_stag = value

    @property
    def g_hot(self):
        return self.hot_gd_inlet_port.linked_connection.g

    @g_hot.setter
    def g_hot(self, value):
        self.hot_gd_inlet_port.linked_connection.g = value

    @property
    def g_cold(self):
        return self.cold_gd_inlet_port.linked_connection.g

    @g_cold.setter
    def g_cold(self, value):
        self.cold_gd_inlet_port.linked_connection.g = value

    @property
    def alpha_hot(self):
        return self.hot_gd_inlet_port.linked_connection.alpha

    @alpha_hot.setter
    def alpha_hot(self, value):
        self.hot_gd_inlet_port.linked_connection.alpha = value

    @property
    def alpha_cold(self):
        return self.cold_gd_inlet_port.linked_connection.alpha

    @alpha_cold.setter
    def alpha_cold(self, value):
        self.cold_gd_inlet_port.linked_connection.alpha = value

    def _check1(self):
        return self.p_stag_cold_in is not None and self.g_hot is not None and self.g_cold is not None\
               and self.T_stag_cold_in is not None and self.T_stag_hot_in is not None\
               and self.T_stag_hot_out is not None

    def _check2(self):
        return self.p_stag_hot_out is not None

    def _compute_output(self):
        self.T_stag_cold_out = self.regeneration_rate * (self.T_stag_hot_in - self.T_stag_cold_in) + \
                               self.T_stag_cold_in
        self.p_stag_cold_out = self.p_stag_cold_in * self.sigma_cold
        self.hot_gd_outlet_port.linked_connection.alpha = self.alpha_hot
        self.cold_gd_outlet_port.linked_connection.alpha = self.alpha_cold
        self.hot_gd_outlet_port.linked_connection.g_fuel = self.hot_gd_inlet_port.linked_connection.g_fuel
        self.cold_gd_outlet_port.linked_connection.g_fuel = self.cold_gd_inlet_port.linked_connection.g_fuel
        self.hot_gd_outlet_port.linked_connection.g = self.g_hot
        self.cold_gd_outlet_port.linked_connection.g = self.g_cold
        self.T_stag_hot_out = self._get_T_stag_hot_out(self.T_stag_cold_in, self.T_stag_cold_out,
                                                       self.T_stag_hot_in, self.T_stag_hot_out)

    def update(self):
        if self._check1():
            self._compute_output()
        if self._check1() and self._check2():
            self._compute_output()
            self.p_stag_hot_in = self.p_stag_hot_out / self.sigma_hot

    def update_connection_current_state(self, relax_coef=1):
        if self._check1() and self._check2():
            self.cold_gd_outlet_port.linked_connection.update_current_state(relax_coef)
            logging.debug('New state of cold_gd_outlet_port connection')
            self.cold_gd_outlet_port.linked_connection.log_state()
            self.hot_gd_outlet_port.linked_connection.update_current_state(relax_coef)
            logging.info('New state of hot_gd_outlet_port connection')
            self.hot_gd_outlet_port.linked_connection.log_state()
            self.hot_gd_inlet_port.linked_connection.update_current_state(relax_coef)
            logging.info('New state of hot_gd_inlet_port connection')
            self.hot_gd_inlet_port.linked_connection.log_state()

    def _get_T_stag_hot_out(self, T_stag_cold_in, T_stag_cold_out, T_stag_hot_in, T_stag_hot_out):
        self.work_fluid_cold.__init__()
        self.work_fluid_hot.__init__()
        self.work_fluid_hot.alpha = self.alpha_hot
        self.work_fluid_cold.T1 = T_stag_cold_in
        self.work_fluid_cold.T2 = T_stag_cold_out
        self.work_fluid_hot.T1 = T_stag_hot_in
        self.work_fluid_hot.T2 = T_stag_hot_out
        c_p_av_ratio = self.work_fluid_cold.c_p_av_int / self.work_fluid_hot.c_p_av_int
        g_rel_ratio = self.g_cold / self.g_hot
        return self.T_stag_hot_in - g_rel_ratio * c_p_av_ratio * (T_stag_cold_out - T_stag_cold_in)


if __name__ == '__main__':

    from optparse import OptionParser
    import sys

    usage = "Usage: turbine_lib [option]"

    parser = OptionParser(usage=usage)

    parser.add_option("-d", "--debug", action="store_true", dest="debug",
                      help="debug mode on")

    parser.add_option("--atmosphere-p0", action="store", type="float", dest="atmosphere_p0", help="Total pressure, default=100000", default=100000)
    parser.add_option("--atmosphere-t0", action="store", type="float", dest="atmosphere_T0", help="Total temperature, default=288", default=288)
    parser.add_option("--inlet-sigma", action="store", type="float", dest="inlet_sigma", help="Pressure ratio out/in, default=0.98", default=0.98)
    parser.add_option("--comp-pi_c", action="store", type="float", dest="comp_pi_c", help="Pressure ratio, default=5", default=5)
    parser.add_option("--comp-eta_stag_p", action="store", type="float", dest="comp_eta_stag_p", help="Polytropic efficiency, default=0.89", default=0.89)
    parser.add_option("--comb_chamber-eta_burn", action="store", type="float", dest="comb_chamber_eta_burn", help="Combustion efficiency, default=0.98", default=0.98)
    parser.add_option("--comb_chamber-Q_n", action="store", type="float", dest="comb_chamber_Q_n", help="Low Heating Value, default=43000000", default=43000000)
    parser.add_option("--comb_chamber-T_stag_out", action="store", type="float", dest="comb_chamber_T_stag_out", help="Output total temperature, default=1600", default=1600)
    parser.add_option("--turbine-eta_stag_p", action="store", type="float", dest="turbine_eta_stag_p", help="Polytropic efficiency, default=0.91", default=0.91)
    parser.add_option("--turbine-p_stag_out", action="store", type="float", dest="turbine_p_stag_out", help="Output total pressure, default=170000", default=170000)
    parser.add_option("--load-power", action="store", type="float", dest="load_power", help="Power demand, default=2300000", default=2300000)
    parser.add_option("--outlet-sigma", action="store", type="float", dest="outlet_sigma", help="Pressure ratio out/in, default=0.99", default=0.99)
    parser.add_option("--regenerator-T_stag_hot_in", action="store", type="float", dest="regenerator_T_stag_hot_in", help="Total temperature of gases at inlet, default=1100", default=1100)
    parser.add_option("--regenerator-T_stag_hot_out", action="store", type="float", dest="regenerator_T_stag_hot_out", help="Total temperature of gases at outlet, default=900", default=900)

    (options, args) = parser.parse_args()

    # if no args or options are provided print help and exit

    if len(sys.argv) == 1 or not any(options.__dict__.values()):
        print("Provide at least one parameter, ie: " + "turbine_lib --atmosphere-t0=288")
        parser.print_help()
        sys.exit(1)

    if options.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.ERROR)

    atmosphere = Atmosphere()
    inlet = Inlet()
    comp = Compressor(pi_c=5)
    comb_chamber = CombustionChamber(Q_n=43e6, l0=14.61)
    turbine = Turbine()
    load = Load(power=2.3e6)
    outlet = Outlet()
    regenerator = Regenerator(regeneration_rate=0.3)
    unit_arr = [comb_chamber, regenerator, atmosphere, inlet, comp, turbine, outlet, load]
    solver = NetworkSolver(unit_arr)
    solver.create_connection(atmosphere.gd_outlet_port, inlet.gd_inlet_port, ConnectionType.GasDynamic)
    solver.create_connection(inlet.gd_outlet_port, comp.gd_inlet_port, ConnectionType.GasDynamic)
    solver.create_connection(comp.gd_outlet_port, regenerator.cold_gd_inlet_port, ConnectionType.GasDynamic)
    solver.create_connection(regenerator.cold_gd_outlet_port, comb_chamber.gd_inlet_port, ConnectionType.GasDynamic)
    solver.create_connection(comb_chamber.gd_outlet_port, turbine.gd_inlet_port, ConnectionType.GasDynamic)
    solver.create_connection(turbine.m_comp_outlet_port, comp.m_inlet_port, ConnectionType.Mechanical)
    solver.create_connection(turbine.gd_outlet_port, regenerator.hot_gd_inlet_port, ConnectionType.GasDynamic)
    solver.create_connection(regenerator.hot_gd_outlet_port, outlet.gd_inlet_port, ConnectionType.GasDynamic)
    solver.create_connection(turbine.m_load_outlet_port, load.m_inlet_port, ConnectionType.Mechanical)
    solver.create_connection(outlet.gd_outlet_port, atmosphere.gd_inlet_port, ConnectionType.GasDynamic)
    comb_chamber.T_stag_out = 1600
    comb_chamber.alpha_out = 2.5
    turbine.p_stag_out = 170e3
    regenerator.T_stag_hot_in = 1100
    regenerator.T_stag_hot_out = 900

    atmosphere.p0 = options.atmosphere_p0
    atmosphere.T0 = options.atmosphere_T0
    inlet.sigma = options.inlet_sigma
    comp.pi_c = options.comp_pi_c
    comp.eta_stag_p = options.comp_eta_stag_p
    comb_chamber.eta_burn= options.comb_chamber_eta_burn
    comb_chamber.Q_n = options.comb_chamber_Q_n
    comb_chamber.T_stag_out = options.comb_chamber_T_stag_out
    turbine.eta_stag_p = options.turbine_eta_stag_p
    turbine.p_stag_out = options.turbine_p_stag_out
    load.power = options.load_power
    outlet.sigma = options.outlet_sigma
    regenerator.T_stag_hot_in = options.regenerator_T_stag_hot_in
    regenerator.T_stag_hot_out = options.regenerator_T_stag_hot_out

    solver.solve()

    print("load.G_air="+str(load.G_air))
    print("comp.T_stag_out="+str(comp.T_stag_out))
    print("comp.p_stag_out="+str(comp.p_stag_out))
    print("turbine.T_stage_out="+str(turbine.T_stag_out))
    print("turbine.p_stage_out="+str(turbine.p_stag_out))














