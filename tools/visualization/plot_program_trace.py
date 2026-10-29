"""Plot commanded program_simulator samples, not hardware measurements.
Requires optional matplotlib. Usage: python plot_program_trace.py trace.csv --output trace.png
"""
import argparse,csv,math
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.suffix.lower() not in {'.png','.svg','.pdf'}:parser.error('output must be PNG, SVG or PDF')
    rows=[]
    with args.trace.open(newline='') as source:
        for row in csv.DictReader(source):
            if len(rows)>=100000:parser.error('plot budget is 100000 samples; decimate longer traces first')
            rows.append(row)
    if not rows:parser.error('empty trace')
    columns={key:[float(row[key]) for row in rows]for key in rows[0]}
    if any(not math.isfinite(x)for values in columns.values()for x in values):parser.error('nonfinite trace')
    times=columns['time_s']
    if any(b<=a for a,b in zip(times,times[1:])):parser.error('time must strictly increase')
    joints=len([key for key in columns if key.startswith('q')and key[1:].isdigit()])
    if not 1<=joints<=12:parser.error('expected 1..12 joints')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(4,3,figsize=(16,12),sharex=True,layout='constrained')
    for order,prefix in enumerate(('q','dq','ddq','jerk')):
        for i in range(joints):axes[order,0].plot(times,columns[prefix+str(i)],label='joint '+str(i))
        axes[order,0].set_title(('Joint position','Joint velocity','Joint acceleration','Joint jerk')[order])
        axes[order,0].set_ylabel(('rad or m','rad/s or m/s','rad/s² or m/s²','rad/s³ or m/s³')[order])
    for key in ('x','y','z'):axes[0,1].plot(times,columns['tcp_'+key],label=key)
    axes[0,1].set_title('TCP position');axes[0,1].set_ylabel('m')
    for key in ('w','x','y','z'):axes[0,2].plot(times,columns['q'+key],label=key)
    axes[0,2].set_title('TCP quaternion [w,x,y,z]')
    for order,prefix in enumerate(('tcp_v','tcp_a','tcp_j'),start=1):
        for i,name in enumerate(('x','y','z')):
            axes[order,1].plot(times,columns[prefix+str(i)],label=name)
            axes[order,2].plot(times,columns[prefix+str(i+3)],label=name)
        axes[order,1].set_title(('TCP linear velocity','TCP linear acceleration','TCP linear jerk')[order-1])
        axes[order,2].set_title(('TCP angular velocity','TCP angular acceleration','TCP angular jerk')[order-1])
        axes[order,1].set_ylabel(('m/s','m/s²','m/s³')[order-1]);axes[order,2].set_ylabel(('rad/s','rad/s²','rad/s³')[order-1])
    for axis in axes.flat:axis.grid(alpha=.25);axis.legend(fontsize=7)
    for axis in axes[-1]:axis.set_xlabel('Program time (s)')
    figure.suptitle('Reconstructed robot-program reference — simulation')
    figure.savefig(args.output,dpi=130);plt.close(figure);print(args.output)
if __name__=='__main__':main()
