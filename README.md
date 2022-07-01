# Qt3DMap

Данная программа представляет собой простую визуальную модель процесса перемещения "континентов" по поверхности сферической земли с возможностью в любой момент времени посмотреть распределение климатических зон на суше по классификации Кёппена. 
Язык программирования: Python.
Используемые библиотеки: PyQt5, NumPy, Numba, Scipy.

Интерфейс программы выглядит следующим образом (Рис. 1).

![image](https://user-images.githubusercontent.com/88488743/176865879-d6757707-da5a-4670-9e4f-5a8aaa3c04d2.png)
(Рис. 1)

Красным отмечены границы континента. 
Кнопка «Старт» запускает процесс движения и столкновения континентов. Кнопка «Климат» останавливает движение и показывает континенты уже с климатическими зонами (Рис. 2). Кнопка «Рестарт» возвращает систему к состоянию на момент запуска. 
Ниже кнопки «Рестарт» находятся ползунки для поворота сферы по соответствующим осям. 
Ниже ползунков идет блок настроек ТМА (Об этом ниже).
• «SEED» – значение, которое будет отправлено генератору случайных чисел.
• «Всего магматических точек» - количество ТМА на сфере. 
• «Максимальная сумма сил», «Максимальная сила точки», «Минимальная сила точки» - задают ограничение для силовых характеристик ТМА.
• «Интервал генерации (мсек)» - время в миллисекундах, после которого система ТМА будет обновлена. 
Ползунок «Коэфф. скорости» задает то, насколько быстро будут двигаться континенты по найденному вектору движения. 

![image](https://user-images.githubusercontent.com/88488743/176869987-f1a97c4c-0ead-4f66-a4b4-97962887c97c.png)
(Рис. 2)

ТМА - точка магматической активности. Это абстрактная сила, придающая определенный вектор движения точкам на поверхности сферы в зависимости от расстояния и угла. Множество ТМА образуют векторное поле планеты. Визуализация этого поля показана на Рис. 3 - для каждого пикселя был вычислен вектор движения, чем больше скорость в этой точке, тем более светлый цвет был присвоен. 

![image](https://user-images.githubusercontent.com/88488743/176870563-4092ec50-37c1-4f5f-a86c-5dc094b9abf3.png)
(Рис. 3)

Столкновение континентов выполнено на базовом уровне.
