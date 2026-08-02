# Слитая картина мира. Публикуется нодой td_world/world_state и является
# ЕДИНСТВЕННЫМ входом диспетчера о том, что происходит на поле.
#
# Любое поле cell может быть "пустым" (name == "") — значит факт ещё не
# установлен. Возраст (*_age_s) позволяет диспетчеру решать, доверять ли
# данным: позиция противника старше нескольких секунд недостоверна.

std_msgs/Header header

Cell rover_cell
Cell drone_cell
Cell target_cell        # где найдена цель
Cell finish_cell        # что прочитано с бумажки
Cell enemy_cell         # где сейчас противник

float32 rover_age_s
float32 drone_age_s
float32 target_age_s
float32 finish_age_s
float32 enemy_age_s

float32 target_confidence
float32 finish_confidence
float32 enemy_confidence

Cell[] blocked_cells    # клетки, запрещённые для ровера прямо сейчас
string phase            # текущая фаза миссии (INIT/SCAN/...)
string[] rationale      # человекочитаемые объяснения последних решений
