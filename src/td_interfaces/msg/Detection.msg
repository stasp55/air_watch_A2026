# Унифицированный результат работы ЛЮБОГО плагина технического зрения.
#
# Восприятие только сообщает факт; решения принимает диспетчер. Поэтому здесь
# нет никаких полей вроде "куда ехать" — только что увидели, где и насколько
# уверенно.

# Что именно найдено
string KIND_TARGET=TARGET              # игрушка-цель (чебурашка)
string KIND_ENEMY=ENEMY                # ровер противника
string KIND_FINISH_LABEL=FINISH_LABEL  # бумажка с финишной клеткой
string KIND_SELF=SELF                  # собственная позиция агента

std_msgs/Header header

string kind
Cell cell                              # клетка, к которой отнесена детекция
geometry_msgs/PoseStamped pose         # опционально, в поле-фрейме (field)
bool has_pose

string payload                         # для FINISH_LABEL — распознанный текст, например "A3"
float32 confidence                     # 0..1
string source                          # "drone/hsv_color", "rover/template_match", ...
