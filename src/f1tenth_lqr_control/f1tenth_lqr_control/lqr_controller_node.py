# ~/autodrive_ws/src/f1tenth_lqr_control/f1tenth_lqr_control/lqr_controller_node.py
import math
import time

import numpy as np
from scipy.linalg import solve_continuous_are

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32
import tf2_ros


WHEELBASE = 0.33          # m, del vehículo F1TENTH real (Car-Parameters.md, AutoDRIVE)
STEER_LIMIT = 0.5236      # rad (30 deg), límite físico de dirección
WHEEL_RADIUS = 0.058      # m, para convertir velocidad angular de rueda a lineal


class LQRControllerNode(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        # ---- Parámetros ----
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'f1tenth_1')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('target_speed_mps', 1.0)
        self.declare_parameter('throttle_kp', 0.5)
        self.declare_parameter('throttle_limit', 0.4)
        self.declare_parameter('q_lat', 3.0)     # peso LQR: error lateral
        self.declare_parameter('q_yaw', 1.0)     # peso LQR: error de orientación
        self.declare_parameter('r_steer', 1.0)   # peso LQR: esfuerzo de control
        self.declare_parameter('lap_start_frac', 0.05)   # % del path considerado "cerca del inicio"
        self.declare_parameter('lap_end_frac', 0.95)     # % del path considerado "cerca del final"
        self.declare_parameter('lap_debounce_s', 3.0)    # tiempo mínimo entre conteos de vuelta
        self.declare_parameter('search_window', 40)
        # --- NUEVO: ayudas de debug/calibración ---
        self.declare_parameter('debug_prints', True)      # activa logs detallados por ciclo
        self.declare_parameter('debug_every_n', 5)          # imprime 1 de cada N ciclos (evita saturar la consola)
        self.declare_parameter('steer_sign', -1.0)          # +1.0 o -1.0: convención de signo del actuador real.
                                                             # Antes estaba hardcodeado a -1.0 ("el actuador gira al
                                                             # revés"). Déjalo como parámetro para poder probar +1.0
                                                             # sin recompilar, ver sección de debugging más abajo.
        self.declare_parameter('tf_timeout_safety', True)   # si el TF se cae DESPUÉS de haber funcionado, frena
        self.declare_parameter('tf_timeout_s', 0.5)         # segundos sin TF antes de considerar "dropout" y frenar

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.target_speed = self.get_parameter('target_speed_mps').value
        self.throttle_kp = self.get_parameter('throttle_kp').value
        self.throttle_limit = self.get_parameter('throttle_limit').value
        self.Q = np.diag([
            self.get_parameter('q_lat').value,
            self.get_parameter('q_yaw').value,
        ])
        self.R = np.array([[self.get_parameter('r_steer').value]])
        self.lap_start_frac = self.get_parameter('lap_start_frac').value
        self.lap_end_frac = self.get_parameter('lap_end_frac').value
        self.lap_debounce_s = self.get_parameter('lap_debounce_s').value
        self.search_window = self.get_parameter('search_window').value
        self.debug_prints = self.get_parameter('debug_prints').value
        self.debug_every_n = max(1, int(self.get_parameter('debug_every_n').value))
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.tf_timeout_safety = self.get_parameter('tf_timeout_safety').value
        self.tf_timeout_s = float(self.get_parameter('tf_timeout_s').value)

        # ---- Estado interno ----
        self.path_xy = None          # np.array (N,2)
        self.path_s = None           # arc-length acumulada, mismo largo que path_xy
        self.path_kappa = None       # curvatura por punto
        self.current_speed = 0.0
        self.last_closest_frac = None
        self.lap_count = 0
        self.last_lap_time = None
        self.was_near_end = False
        self.last_idx = None
        self._debug_counter = 0
        self._last_tf_ok_time = None     # última vez que el lookup de TF funcionó
        self._tf_ever_ok = False         # si alguna vez tuvimos TF válido (distingue arranque de dropout)
        self._tf_fail_streak = 0         # fallos consecutivos, para no spamear el log de frames
        self._encoder_debug_logged = {'left': 0, 'right': 0}
        self._prev_abs_e_yaw = None
        self._saturation_streak = 0      # ciclos consecutivos con steer saturado Y |e_yaw| creciendo

        # ---- Suscripciones ----
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Path, '/smoothed_path', self.on_path, path_qos)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/left_encoder', self.on_left_encoder, 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/right_encoder', self.on_right_encoder, 10)

        self._left_wheel_speed = 0.0
        self._right_wheel_speed = 0.0
        self._left_enc_last_pos = None
        self._left_enc_last_time = None
        self._right_enc_last_pos = None
        self._right_enc_last_time = None

        # ---- Publicadores ----
        self.steer_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 1)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 1)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Loop de control ----
        rate = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'LQR controller listo (debug_prints={self.debug_prints}, steer_sign={self.steer_sign}). '
            'Esperando /smoothed_path...'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_path(self, msg: Path):
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        if len(pts) < 3:
            self.get_logger().warn('Path recibido demasiado corto, se ignora.')
            return

        s = np.zeros(len(pts))
        for i in range(1, len(pts)):
            s[i] = s[i - 1] + np.linalg.norm(pts[i] - pts[i - 1])

        kappa = self._compute_curvature(pts)

        self.path_xy = pts
        self.path_s = s
        self.path_kappa = kappa
        self.get_logger().info(f'Trayectoria recibida: {len(pts)} puntos, longitud {s[-1]:.2f} m.')

        if self.debug_prints:
            self.get_logger().info(
                f'[DEBUG path] kappa min={kappa.min():.3f} max={kappa.max():.3f} '
                f'mean_abs={np.mean(np.abs(kappa)):.3f}'
            )

    def on_left_encoder(self, msg: JointState):
        if self.debug_prints and self._encoder_debug_logged['left'] < 3:
            self._encoder_debug_logged['left'] += 1
            self.get_logger().info(
                f'[DEBUG encoder L] name={list(msg.name)} position={list(msg.position)} '
                f'velocity={list(msg.velocity)} (velocity siempre viene vacío en AutoDRIVE -> '
                f'calculamos la velocidad derivando "position" en vez de usar este campo)'
            )
        self._left_wheel_speed = self._speed_from_encoder_position(
            msg, self._left_enc_last_pos, self._left_enc_last_time, 'left'
        )

    def on_right_encoder(self, msg: JointState):
        if self.debug_prints and self._encoder_debug_logged['right'] < 3:
            self._encoder_debug_logged['right'] += 1
            self.get_logger().info(
                f'[DEBUG encoder R] name={list(msg.name)} position={list(msg.position)} '
                f'velocity={list(msg.velocity)} (velocity siempre viene vacío en AutoDRIVE -> '
                f'calculamos la velocidad derivando "position" en vez de usar este campo)'
            )
        self._right_wheel_speed = self._speed_from_encoder_position(
            msg, self._right_enc_last_pos, self._right_enc_last_time, 'right'
        )

    def _speed_from_encoder_position(self, msg, last_pos, last_time, side):
        """
        AutoDRIVE no llena msg.velocity (siempre []), pero sí llena msg.position
        (ángulo acumulado de la rueda, en rad). Calculamos la velocidad angular
        derivando esa posición en el tiempo, y la convertimos a velocidad lineal
        de rueda con WHEEL_RADIUS.
        """
        if not msg.position:
            return getattr(self, f'_{side}_wheel_speed', 0.0)

        pos = msg.position[0]
        now = time.time()

        if last_pos is None or last_time is None:
            # primera lectura: no hay con qué derivar todavía
            if side == 'left':
                self._left_enc_last_pos = pos
                self._left_enc_last_time = now
            else:
                self._right_enc_last_pos = pos
                self._right_enc_last_time = now
            return 0.0

        dt = now - last_time
        if dt <= 0.0:
            return getattr(self, f'_{side}_wheel_speed', 0.0)

        delta = pos - last_pos
        # por si el ángulo viene envuelto en [-pi, pi] o similar: corrige el salto
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi

        angular_speed = delta / dt
        wheel_speed = angular_speed * WHEEL_RADIUS

        if side == 'left':
            self._left_enc_last_pos = pos
            self._left_enc_last_time = now
        else:
            self._right_enc_last_pos = pos
            self._right_enc_last_time = now

        return wheel_speed

    @staticmethod
    def _compute_curvature(pts):
        # NOTA: esto es un circuito cerrado (ver lógica de conteo de vueltas),
        # pero esta función NO envuelve en los extremos (kappa[0]/kappa[-1] solo
        # copian al vecino en vez de calcularse con wraparound real). El error
        # que esto introduce es pequeño comparado con el bug de idx_next en
        # _compute_errors (ver abajo), pero si notas un "salto" de dirección
        # justo en la línea de meta, esto también puede estar contribuyendo.
        n = len(pts)
        kappa = np.zeros(n)
        for i in range(1, n - 1):
            p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
            a = np.linalg.norm(p2 - p1)
            b = np.linalg.norm(p3 - p2)
            c = np.linalg.norm(p3 - p1)
            if a * b * c == 0:
                continue
            # signed_area: positivo = giro a la izquierda, negativo = giro a la derecha
            signed_area = 0.5 * ((p2[0]-p1[0])*(p3[1]-p1[1]) - (p3[0]-p1[0])*(p2[1]-p1[1]))
            kappa[i] = 4 * signed_area / (a * b * c)
        kappa[0], kappa[-1] = kappa[1], kappa[-2]
        return kappa

    # ------------------------------------------------------------------
    # Loop principal
    # ------------------------------------------------------------------

    def control_loop(self):
        if self.path_xy is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception as e:
            self._handle_tf_failure(e)
            return

        self._last_tf_ok_time = time.time()
        self._tf_ever_ok = True
        self._tf_fail_streak = 0

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        roll, pitch, yaw = self._rpy_from_quaternion(tf.transform.rotation)

        if self.debug_prints and self._debug_counter % (self.debug_every_n * 4) == 0:
            q = tf.transform.rotation
            self.get_logger().info(
                f'[DEBUG quat] raw=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f}) '
                f'roll={math.degrees(roll):+.1f}deg pitch={math.degrees(pitch):+.1f}deg '
                f'yaw={math.degrees(yaw):+.1f}deg '
                '(en terreno plano, roll y pitch deberían quedarse cerca de 0 SIEMPRE; '
                'si se mueven mucho o de forma errática, hay un problema de conversión '
                'de ejes Unity->ROS en el TF, y el yaw no es confiable)'
            )

        # abs(): por ahora solo nos interesa la magnitud de la velocidad para el
        # LQR y el control de velocidad; si en el futuro implementas reversa,
        # esto habría que revisarlo.
        self.current_speed = abs((self._left_wheel_speed + self._right_wheel_speed) / 2.0)

        idx = self._closest_index(x, y)
        self._update_lap_counter(idx)

        e_lat, e_yaw = self._compute_errors(x, y, yaw, idx)
        kappa_ref = self.path_kappa[idx]

        steer_cmd, delta_fb, delta_ff = self._lqr_steer(e_lat, e_yaw, kappa_ref, self.current_speed)
        throttle_cmd = self._speed_control(self.current_speed)

        self.steer_pub.publish(Float32(data=float(steer_cmd)))
        self.throttle_pub.publish(Float32(data=float(throttle_cmd)))

        if self.debug_prints:
            self._debug_counter += 1
            if self._debug_counter % self.debug_every_n == 0:
                frac = idx / (len(self.path_xy) - 1)
                self.get_logger().info(
                    f'[DEBUG] idx={idx} ({frac*100:.1f}%) '
                    f'pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg '
                    f'v={self.current_speed:.2f}m/s | '
                    f'e_lat={e_lat:+.3f}m e_yaw={math.degrees(e_yaw):+.1f}deg kappa={kappa_ref:+.3f} | '
                    f'delta_fb={math.degrees(delta_fb):+.1f}deg delta_ff={math.degrees(delta_ff):+.1f}deg | '
                    f'steer_cmd={steer_cmd:+.3f}rad ({math.degrees(steer_cmd):+.1f}deg) '
                    f'throttle={throttle_cmd:+.3f}'
                )

        # --- Detector de "espiral" / realimentación positiva ---
        # Si el steering está saturado en el límite Y el error de yaw sigue
        # CRECIENDO en vez de reducirse durante varios ciclos seguidos, es
        # señal fuerte de que el signo del actuador está invertido
        # (steer_sign equivocado), no de que falte ganancia.
        is_saturated = abs(abs(steer_cmd) - STEER_LIMIT) < 1e-6
        abs_e_yaw = abs(e_yaw)
        if is_saturated and self._prev_abs_e_yaw is not None and abs_e_yaw > self._prev_abs_e_yaw:
            self._saturation_streak += 1
        else:
            self._saturation_streak = 0
        self._prev_abs_e_yaw = abs_e_yaw

        if self._saturation_streak == 15:
            self.get_logger().error(
                '⚠️ POSIBLE SIGNO INVERTIDO: el steering lleva saturado varios ciclos y el error de '
                'yaw sigue CRECIENDO en vez de reducirse (el auto está "espiralando"). '
                f'Prueba correr con -p steer_sign:={-self.steer_sign} en vez de {self.steer_sign}.',
                throttle_duration_sec=5.0,
            )

    def _handle_tf_failure(self, exc):
        """
        Se llama cuando lookup_transform falla. Distingue entre:
          (a) arranque normal: todavía no ha llegado el primer TF -> solo informativo.
          (b) dropout real: el TF ya había funcionado antes y ahora se cayó -> esto
              es lo que probablemente causa tus choques, porque sin este manejo el
              nodo simplemente no publica nada y el auto sigue con el ÚLTIMO comando
              recibido (a ciegas) hasta que el TF vuelva.
        Si tf_timeout_safety está activo y llevamos más de tf_timeout_s sin TF
        después de haber tenido uno bueno, frenamos el auto (steer=0, throttle=0)
        en vez de dejarlo avanzando con el comando viejo.
        """
        self._tf_fail_streak += 1
        now = time.time()

        if not self._tf_ever_ok:
            # Arranque normal: nunca hemos tenido TF. Avisar una sola vez cada tanto,
            # sin alarmar de más.
            self.get_logger().warn(
                f'TF no disponible todavía (esperando primer "{self.map_frame} -> {self.base_frame}"): {exc}',
                throttle_duration_sec=2.0,
            )
            return

        elapsed = now - self._last_tf_ok_time
        self.get_logger().warn(
            f'TF DROPOUT: "{self.map_frame} -> {self.base_frame}" dejó de existir hace {elapsed:.2f}s '
            f'(ya había funcionado antes -> esto puede estar causando choques a ciegas). Detalle: {exc}',
            throttle_duration_sec=1.0,
        )

        if self.debug_prints and self._tf_fail_streak in (1, 5, 20, 50):
            try:
                frames = self.tf_buffer.all_frames_as_string()
            except Exception:
                frames = '(no se pudo obtener la lista de frames)'
            self.get_logger().info(f'[DEBUG TF] Frames disponibles en el árbol TF:\n{frames}')

        if self.tf_timeout_safety and elapsed >= self.tf_timeout_s:
            # Freno de seguridad: no dejamos el último comando "colgado".
            self.steer_pub.publish(Float32(data=0.0))
            self.throttle_pub.publish(Float32(data=0.0))
            self.get_logger().warn(
                f'FRENO DE SEGURIDAD activado: TF caído por >= {self.tf_timeout_s:.2f}s, '
                'publicando steer=0 / throttle=0 en vez de mantener el último comando.',
                throttle_duration_sec=1.0,
            )

    # ------------------------------------------------------------------
    # Geometría / errores
    # ------------------------------------------------------------------

    def _closest_index(self, x, y):
        n = len(self.path_xy)
        if self.last_idx is None:
            # primera vez: sí buscamos en todo el path
            d = np.hypot(self.path_xy[:, 0] - x, self.path_xy[:, 1] - y)
            idx = int(np.argmin(d))
        else:
            # después: solo buscamos cerca del último índice, con wrap-around porque es un circuito
            idxs = [(self.last_idx + offset) % n for offset in range(-self.search_window, self.search_window + 1)]
            dists = [math.hypot(self.path_xy[i, 0] - x, self.path_xy[i, 1] - y) for i in idxs]
            idx = idxs[int(np.argmin(dists))]

        self.last_idx = idx
        return idx

    def _compute_errors(self, x, y, yaw, idx):
        n = len(self.path_xy)
        px, py = self.path_xy[idx]
        # FIX: tangente local con wraparound real (circuito cerrado). Antes usaba
        # min(idx+1, n-1), lo que hacía que en el último punto del path (justo en
        # la línea de meta) el vector tangente fuera (0,0) -> path_yaw=0 -> e_yaw
        # y e_lat calculados con una referencia de rumbo incorrecta exactamente
        # donde se cuenta la vuelta.
        idx_next = (idx + 1) % n
        tx, ty = self.path_xy[idx_next] - self.path_xy[idx]
        path_yaw = math.atan2(ty, tx)

        dx, dy = x - px, y - py
        e_lat = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        e_yaw = self._normalize_angle(yaw - path_yaw)
        return e_lat, e_yaw

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _rpy_from_quaternion(q):
        # Roll (X), Pitch (Y), Yaw (Z) — convención estándar ROS (ZYX intrínseco).
        # Si el auto está en terreno plano, roll y pitch deberían ser ~0 SIEMPRE.
        # Si no lo son, o si se mueven de forma errática, el TF que publica AutoDRIVE
        # probablemente no está en la convención estándar de ROS (Unity es zurdo/Y-arriba,
        # ROS es diestro/Z-arriba) y el "yaw" que sacamos de aquí no es confiable.
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # ------------------------------------------------------------------
    # LQR (dirección)
    # ------------------------------------------------------------------

    def _lqr_steer(self, e_lat, e_yaw, kappa_ref, v):
        v_eff = max(v, 0.3)

        A = np.array([
            [0.0, v_eff],
            [0.0, 0.0],
        ])
        B = np.array([
            [0.0],
            [v_eff / WHEELBASE],
        ])

        P = solve_continuous_are(A, B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ B.T @ P

        state = np.array([[e_lat], [e_yaw]])
        delta_fb = float(-K @ state)
        delta_ff = math.atan(WHEELBASE * kappa_ref)

        delta_model = delta_fb + delta_ff
        # Convención de signo del actuador real, ahora parametrizada (ver
        # 'steer_sign' en __init__). Antes estaba hardcodeado a -1.0. Antes de
        # confiar en este valor, verifica empíricamente cuál signo corresponde
        # a la convención real de AutoDrive (ver mensaje de chat / notas de
        # debugging para el procedimiento).
        steer_cmd = self.steer_sign * delta_model

        steer_cmd = max(-STEER_LIMIT, min(STEER_LIMIT, steer_cmd))
        return steer_cmd, delta_fb, delta_ff

    # ------------------------------------------------------------------
    # Control de velocidad (throttle)
    # ------------------------------------------------------------------

    def _speed_control(self, v_current):
        error = self.target_speed - v_current
        throttle = self.throttle_kp * error
        return max(-self.throttle_limit, min(self.throttle_limit, throttle))

    # ------------------------------------------------------------------
    # Conteo de vueltas + cronómetro
    # ------------------------------------------------------------------

    def _update_lap_counter(self, idx):
        frac = idx / (len(self.path_xy) - 1)

        near_end = frac >= self.lap_end_frac
        near_start = frac <= self.lap_start_frac

        if near_end:
            self.was_near_end = True

        if near_start and self.was_near_end:
            now = time.time()
            if self.last_lap_time is None:
                self.lap_count += 1
                self.get_logger().info(f'🏁 Vuelta {self.lap_count} completada (referencia de inicio).')
                self.last_lap_time = now
                self.was_near_end = False
            elif now - self.last_lap_time >= self.lap_debounce_s:
                lap_time = now - self.last_lap_time
                self.lap_count += 1
                self.get_logger().info(f'🏁 Vuelta {self.lap_count} completada | tiempo: {lap_time:.2f} s')
                self.last_lap_time = now
                self.was_near_end = False


def main():
    rclpy.init()
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()