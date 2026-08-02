# Периодический отчёт исполнителя (ровер/дрон) о себе.

std_msgs/Header header

string agent_id          # "rover-01", "drone-01"
string state             # IDLE / BUSY / ERROR / OFFLINE
string current_action    # имя выполняемого навыка, "" если простаивает
Cell cell                # текущая клетка агента
float32 battery          # 0..1, -1 если неизвестно
bool localized           # есть ли валидная привязка к полю
string message           # свободный текст для логов/защиты
