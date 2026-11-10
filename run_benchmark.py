#!/usr/bin/env python3
"""Run sequentially: fresh process per manager/size/seed; no parallel timing."""
import csv, io, random, subprocess, statistics, pathlib
root = pathlib.Path(__file__).resolve().parent
fields = 'manager,n,seed,workload,scene_us,init_us,scene_heap_bytes,index_heap_bytes,rss_base,rss_scene,rss_index,query_us,batch_min_us,batch_max_us,candidates,hits'.split(',')
jobs = [(m,n,s) for m in ['naive','sap','ssap','interval','tree','tree_array','hash'] for n in [1,10,100,1000,10000] for s in range(42,47)]
random.Random(2026).shuffle(jobs)
rows=[]
with (root/'results/raw.csv').open('w') as f:
    writer=csv.writer(f); writer.writerow(fields)
    for k,(m,n,s) in enumerate(jobs):
        result=subprocess.run([str(root/'build/coal_benchmark'),m,str(n),str(s)],text=True,capture_output=True,check=True)
        batch=list(csv.reader(io.StringIO(result.stdout)))
        assert len(batch)==3
        writer.writerows(batch); f.flush()
        rows.extend(dict(zip(fields,r)) for r in batch)
        if (k+1)%35==0: print(f'{k+1}/{len(jobs)} configurations passed',flush=True)
summary_fields=['manager','n','workload','scene_us','init_us','scene_heap_bytes','index_heap_bytes','query_us','query_min_seed_us','query_max_seed_us','candidates','hits']
with (root/'results/summary.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=summary_fields); w.writeheader()
    for m in ['naive','sap','ssap','interval','tree','tree_array','hash']:
        for n in [1,10,100,1000,10000]:
            for workload in ['outside_miss','hit','random']:
                group=[r for r in rows if r['manager']==m and int(r['n'])==n and r['workload']==workload]
                out={key:statistics.median(float(r[key]) for r in group) for key in ['scene_us','init_us','scene_heap_bytes','index_heap_bytes','query_us','candidates','hits']}
                q=[float(r['query_us']) for r in group]
                out.update(manager=m,n=n,workload=workload,query_min_seed_us=min(q),query_max_seed_us=max(q))
                w.writerow(out)
print('Saved raw.csv and summary.csv',flush=True)
