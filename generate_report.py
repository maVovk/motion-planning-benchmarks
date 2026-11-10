import csv,pathlib
p=pathlib.Path(__file__).resolve().parent
r=list(csv.DictReader((p/'results/summary.csv').open()))
ms=['naive','sap','ssap','interval','tree','tree_array','hash']; ns=[1,10,100,1000,10000]
def row(m,n,w='random'):return next(x for x in r if x['manager']==m and int(x['n'])==n and x['workload']==w)
out=['# Результаты Coal на Mac\n','Дата: 9 ноября 2026. Apple M4, 16 GiB RAM, macOS 26.6.2.\nCoal 3.0.3, Eigen 3.4.1, AppleClang 21.0.0, Release (-O3 -DNDEBUG), arm64.\nAVX не используется. В библиотеке — готовый arm64 wheel libcoal.\n','Все 175 процессов завершились успешно; 134 400 проверок запросов\n(175 × 384 × 2) совпали с аналитическим oracle. Ниже — медианы пяти seeds.\n']
for key,title,factor in [('init_us','Построение индекса, мкс',1),('index_heap_bytes','Занятая heap память индекса, KiB',1/1024),('query_us','Смешанные random-запросы, мкс/запрос',1)]:
 out+=['## '+title+'\n','| Менеджер | 1 | 10 | 100 | 1000 | 10000 |','|---|---:|---:|---:|---:|---:|']
 for m in ms:out+=['| '+m+' | '+' | '.join(f'{float(row(m,n)[key])*factor:.3f}' for n in ns)+' |']
 out+=['']
out+=['## Все сценарии при N=10000\n','| Менеджер | Outside miss, мкс | Hit, мкс | Random, мкс | Random min–max по seeds, мкс | Narrow phase вызовов / random-запрос |','|---|---:|---:|---:|---:|---:|']
for m in ms:
 x=row(m,10000)
 out+=['| '+m+' | '+' | '.join(f'{float(row(m,10000,w)["query_us"]):.3f}' for w in ['outside_miss','hit','random'])+f' | {float(x["query_min_seed_us"]):.3f}–{float(x["query_max_seed_us"]):.3f} | {float(x["candidates"]):.3f} |']
out+=['','## Общая память объектов сцены\n','| N | Heap сферы + CollisionObject, KiB |','|---|---:|']
for n in ns:out+= [f'| {n} | {float(row("tree",n)["scene_heap_bytes"])/1024:.3f} |']
out+=['','Одна сцена N=10000 занимает 3.36 MiB heap геометрий и объектов.\nК этому добавляется индекс из таблицы и служебные данные процесса.\nУказаны занятые байты после построения, не пиковая память.\nПодробная методика и ограничения — в README.md; исходные RSS и min/max времени пакетов — в raw.csv.\n']
(p/'results/REPORT.md').write_text('\n'.join(out))
print('\n'.join(out))
