from pg_extensions import *
import json


class Settings:
    G = 0

    PAUSED = False
    DEBUG = False
    SHOW_NAMES = False
    GRID_SPACING = 50
    UNZOOMED_GRID_SPACING_MULTIPLIER = 10

    camera_position = Vector2()
    zoom = 1

    INF = 2**16  # not actually infinity, just a big number


class Body:
    def __init__(
        self,
        name="BODY",
        position=Vector2(0, 0),
        velocity=Vector2(0, 0),
        radius=25,
        mass=1,
        color=WHITE,
        width=0,
    ):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.radius = radius
        self.mass = mass
        self.color = color
        self.width = width

        self.total_force = Vector2()

    def calculate_force(self, other):
        delta = other.position - self.position
        if delta.magnitude() != 0:
            dir = delta.normalize()
            force = Settings.G * self.mass * other.mass / delta.sqr_magnitude()  # F=G*m1*m2/r²
            return dir * force
        else:
            return Vector2.random_polar(0, math.tau, 50, 200)  # launch in random direction to prevent bunching

    def update(self, bodies, barycenter):
        if not Settings.PAUSED:
            # loops over all bodies and adds together all the forces being applied on this body
            self.total_force = Vector2()
            for other in bodies:
                if self == other:
                    continue

                self.total_force += self.calculate_force(other)

            # runge-kutta 4 implementation (more accurate position and velocity updating)
            self.acceleration = self.total_force / self.mass

            self.position, self.velocity = runge_kutta_4(self.position, self.velocity, self.acceleration, window.delta_time)

        self.render(barycenter)

    def render(self, barycenter):
        draw_circle(
            window.SURFACE,
            self.color,
            self.position * Settings.zoom + Settings.camera_position,
            clamp(self.radius * Settings.zoom, 5, math.inf),
            self.width,
        )

        if Settings.DEBUG:
            draw_line(
                window.SURFACE,
                GREEN,
                self.position * Settings.zoom + Settings.camera_position,
                (self.position + self.velocity) * Settings.zoom + Settings.camera_position,
                2,
            )
            draw_line(
                window.SURFACE,
                RED,
                self.position * Settings.zoom + Settings.camera_position,
                (self.position + self.total_force.normalize() * self.radius * 3) * Settings.zoom + Settings.camera_position,
                2,
            )
            draw_line(
                window.SURFACE,
                WHITE,
                self.position * Settings.zoom + Settings.camera_position,
                barycenter * Settings.zoom + Settings.camera_position,
                2,
            )

        if Settings.SHOW_NAMES:
            name_text = Text(
                f"{self.name}",
                Text.arial_24,
                Vector2(self.position.x + 32, self.position.y + 16) * Settings.zoom + Settings.camera_position,
                Text.bottom_left,
                WHITE,
                BLACK,
            )
            name_text.render()


def compute_barycenter(bodies):
    total_mass = sum(body.mass for body in bodies)

    if total_mass == 0:
        return Vector2(0, 0)  # avoid division by zero

    barycenter = sum((body.position * body.mass for body in bodies), Vector2()) / total_mass
    return barycenter


def update_ui():
    # top right (stats)
    if Settings.DEBUG:
        fps_text = Text(
            f"FPS: {window.clock.get_fps():.2f}",
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 1),
            Text.top_left,
            WHITE,
            BLACK,
        )
        fps_text.render()

        dt_text = Text(
            f"FRAME TIME / TIMESTEP: {window.delta_time:.3f}s",
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 2),
            Text.top_left,
            WHITE,
            BLACK,
        )
        dt_text.render()

        elapsed_time_text = Text(
            f"ELAPSED TIME: {elapsed_time:.3f}s",
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 3),
            Text.top_left,
            WHITE,
            BLACK,
        )
        elapsed_time_text.render()

        current_frame_text = Text(
            f"FRAME: {current_frame}",
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 4),
            Text.top_left,
            WHITE,
            BLACK,
        )
        current_frame_text.render()

        ob_count_text = Text(
            f"OBJECT COUNT: {len(bodies)}",
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, window.HEIGHT // 2 - 32 * 5),
            Text.top_left,
            WHITE,
            BLACK,
        )
        ob_count_text.render()

        # bottom left (controls)
        paused_text = Text(
            f"PAUSED (SPACEBAR): {Settings.PAUSED}".upper(),
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, -window.HEIGHT // 2 + 32 * 5),
            Text.bottom_left,
            WHITE,
            BLACK,
        )
        paused_text.render()

        debug_text = Text(
            f"DEBUG MODE (1): {Settings.DEBUG}".upper(),
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, -window.HEIGHT // 2 + 32 * 4),
            Text.bottom_left,
            WHITE,
            BLACK,
        )
        debug_text.render()

        show_names_text = Text(
            f"SHOW NAMES (2): {Settings.SHOW_NAMES}".upper(),
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, -window.HEIGHT // 2 + 32 * 3),
            Text.bottom_left,
            WHITE,
            BLACK,
        )
        show_names_text.render()

        cam_pos_text = Text(
            f"CAMERA POS (MOUSE): {Settings.camera_position}",
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, -window.HEIGHT // 2 + 32 * 2),
            Text.bottom_left,
            WHITE,
            BLACK,
        )
        cam_pos_text.render()

        zoom_text = Text(
            f"ZOOM (+/-): {Settings.zoom:.2f}",
            Text.arial_24,
            Vector2(-window.WIDTH // 2 + 32, -window.HEIGHT // 2 + 32 * 1),
            Text.bottom_left,
            WHITE,
            BLACK,
        )
        zoom_text.render()


def start():
    global bodies, elapsed_time, current_frame
    elapsed_time = 0.0
    current_frame = 0

    # load config from config.json
    with open("config.json", "r") as f:
        contents = f.read()
        config = json.loads(contents)
        settings_info = config["settings"]
        bodies_info = config["bodies"]

        Settings.G = settings_info["G"]
        Settings.PAUSED = settings_info["paused_at_start"]
        Settings.DEBUG = settings_info["debug_mode"]
        Settings.SHOW_NAMES = settings_info["show_names"]

        bodies = []

        for i in range(len(bodies_info)):
            bodies.append(
                Body(
                    bodies_info[i]["name"],
                    Vector2(bodies_info[i]["position"]["x"], bodies_info[i]["position"]["y"]),
                    Vector2(bodies_info[i]["velocity"]["x"], bodies_info[i]["velocity"]["y"]),
                    bodies_info[i]["radius"],
                    bodies_info[i]["mass"],
                    Color(
                        bodies_info[i]["color"]["r"],
                        bodies_info[i]["color"]["g"],
                        bodies_info[i]["color"]["b"],
                    ),
                    bodies_info[i]["width"],
                )
            )


def update():
    global window, elapsed_time, current_frame
    window = get_window()
    window.clear(BLACK)

    if not Settings.PAUSED:
        elapsed_time += window.delta_time
        current_frame += 1

    if input_manager.get_key_down(pygame.K_ESCAPE):
        window.running = False
    if input_manager.get_key_down(pygame.K_SPACE):
        Settings.PAUSED = not Settings.PAUSED
    if input_manager.get_key_down(pygame.K_1):
        Settings.DEBUG = not Settings.DEBUG
    if input_manager.get_key_down(pygame.K_2):
        Settings.SHOW_NAMES = not Settings.SHOW_NAMES
    if input_manager.get_mouse_held(0):
        # camera movement
        mouse_movement = input_manager.get_mouse_motion()
        Settings.camera_position.x += mouse_movement.x
        Settings.camera_position.y -= mouse_movement.y
    if input_manager.get_key_held(pygame.K_EQUALS) and not input_manager.get_key_held(pygame.K_MINUS):
        Settings.zoom += Settings.zoom / 50  # increase zoom
    if not input_manager.get_key_held(pygame.K_EQUALS) and input_manager.get_key_held(pygame.K_MINUS):
        Settings.zoom -= Settings.zoom / 50  # decrease zoom

    Settings.zoom += Settings.zoom / 10 * input_manager.get_mouse_wheel().y

    Settings.zoom = clamp(Settings.zoom, 0.05, 10)

    # calculate barycenter
    barycenter = compute_barycenter(bodies)

    # draw grid
    # x-axis line
    draw_line(
        window.SURFACE,
        WHITE,
        Vector2(-Settings.INF, 0) * Settings.zoom + Settings.camera_position,
        Vector2(Settings.INF, 0) * Settings.zoom + Settings.camera_position,
        math.ceil(Settings.zoom * 2) if Settings.zoom > 0.5 else 2,
    )

    # y-axis line
    draw_line(
        window.SURFACE,
        WHITE,
        Vector2(0, -Settings.INF) * Settings.zoom + Settings.camera_position,
        Vector2(0, Settings.INF) * Settings.zoom + Settings.camera_position,
        math.ceil(Settings.zoom * 2) if Settings.zoom > 0.5 else 2,
    )

    # horizontal lines
    if Settings.zoom > 0.33:
        for y in range(0, Settings.INF // 2, Settings.GRID_SPACING):
            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(-Settings.INF, y) * Settings.zoom + Settings.camera_position,
                Vector2(Settings.INF, y) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )

            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(-Settings.INF, -y) * Settings.zoom + Settings.camera_position,
                Vector2(Settings.INF, -y) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )
    else:
        for y in range(0, Settings.INF // 2, Settings.GRID_SPACING * Settings.UNZOOMED_GRID_SPACING_MULTIPLIER):
            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(-Settings.INF, y) * Settings.zoom + Settings.camera_position,
                Vector2(Settings.INF, y) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )

            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(-Settings.INF, -y) * Settings.zoom + Settings.camera_position,
                Vector2(Settings.INF, -y) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )

    # vertical lines
    if Settings.zoom > 0.33:
        for x in range(0, Settings.INF // 2, Settings.GRID_SPACING):
            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(x, -Settings.INF) * Settings.zoom + Settings.camera_position,
                Vector2(x, Settings.INF) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )

            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(-x, -Settings.INF) * Settings.zoom + Settings.camera_position,
                Vector2(-x, Settings.INF) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )
    else:
        for x in range(0, Settings.INF // 2, Settings.GRID_SPACING * Settings.UNZOOMED_GRID_SPACING_MULTIPLIER):
            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(x, -Settings.INF) * Settings.zoom + Settings.camera_position,
                Vector2(x, Settings.INF) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )

            draw_line(
                window.SURFACE,
                WHITE,
                Vector2(-x, -Settings.INF) * Settings.zoom + Settings.camera_position,
                Vector2(-x, Settings.INF) * Settings.zoom + Settings.camera_position,
                math.ceil(Settings.zoom),
            )

    # update bodies
    for body in bodies:
        body.update(bodies, barycenter)

    # draw barycenter
    if Settings.DEBUG:
        draw_circle(window.SURFACE, WHITE, barycenter * Settings.zoom + Settings.camera_position, 5)

    update_ui()

    set_window(window)


if __name__ == "__main__":
    run(start, update, 1280, 720, False, "Customizable Gravity Simulation")
