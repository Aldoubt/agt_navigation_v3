"""Aggregate explicit run outcomes and independently labelled row/recovery errors."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def quantile(values, p):
    values=sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    index=(len(values)-1)*p
    lower=int(index); upper=min(lower+1,len(values)-1)
    return values[lower]+(values[upper]-values[lower])*(index-lower)


def series_summary(values):
    values=[float(v) for v in values if math.isfinite(float(v))]
    return {'n':len(values),'mean':statistics.fmean(values) if values else None,
            'median':quantile(values,.5),'p95':quantile(values,.95)}


def wilson_interval(successes,total):
    if not total: return [None,None]
    z=1.959963984540054
    p=successes/total
    center=(p+z*z/(2*total))/(1+z*z/total)
    half=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/(1+z*z/total)
    return [max(0,center-half),min(1,center+half)]


def row_metrics(path):
    values={'lateral_error_m':[],'heading_error_rad':[],'width_error_m':[],'latency_ms':[]}
    valid, total=0,0
    with Path(path).open() as stream:
        for row in csv.DictReader(stream):
            total+=1
            if row.get('row_valid','').lower() not in ('true','1'):
                continue
            valid+=1
            for key,predicted,reference in (
                ('lateral_error_m','lateral_error_m','reference_lateral_error_m'),
                ('heading_error_rad','heading_error_rad','reference_heading_error_rad'),
                ('width_error_m','width_m','reference_width_m')):
                if row.get(reference) and row.get(predicted):
                    error=float(row[predicted])-float(row[reference])
                    if key=='heading_error_rad':
                        error=math.atan2(math.sin(error),math.cos(error))
                    values[key].append(abs(error))
            if row.get('latency_ms'):
                values['latency_ms'].append(float(row['latency_ms']))
    return {'frames':total,'valid_frames':valid,'valid_rate':valid/total if total else None,
            'reference_required':True,**{key:series_summary(value) for key,value in values.items()}}


def run_metrics(path):
    with Path(path).open() as stream:
        runs=list(csv.DictReader(stream))
    required={'run_id','group','route_id','growth_stage','outcome','reason','duration_sec',
              'localization_stop_sec','interventions','map_hash','parameter_hash','reference_source'}
    if not runs or not required.issubset(runs[0]):
        raise ValueError('runs CSV missing required columns: '+', '.join(sorted(required)))
    if len({r['run_id'] for r in runs}) != len(runs):
        raise ValueError('duplicate run_id')
    groups={}
    for run in runs:
        if any(not run[k].strip() for k in ('run_id','group','route_id','growth_stage','reason','reference_source')):
            raise ValueError('every attempt needs identity, reason and reference provenance')
        duration,stopped=float(run['duration_sec']),float(run['localization_stop_sec'])
        if not math.isfinite(duration) or not math.isfinite(stopped) or duration<0 or stopped<0 or stopped>duration:
            raise ValueError('finite nonnegative times required; localization stop cannot exceed duration')
        if int(run['interventions'])<0: raise ValueError('nonnegative interventions required')
        if run['outcome'] not in ('completed','algorithm_failed','environment_infeasible','sensor_failed','operator_aborted'):
            raise ValueError('explicit outcome classification required')
        if not run['map_hash'] or not run['parameter_hash']:
            raise ValueError('frozen identities required for every attempt')
        key='|'.join(run[k] for k in ('group','route_id','growth_stage'))
        groups.setdefault(key,[]).append(run)
    result={}
    for key,items in groups.items():
        outcome={name:sum(r['outcome']==name for r in items) for name in
                 ('completed','algorithm_failed','environment_infeasible','sensor_failed','operator_aborted')}
        result[key]={'attempts':len(items),'outcomes':outcome,
                     'completion_rate':outcome['completed']/len(items),
                     'completion_rate_95ci_wilson':wilson_interval(outcome['completed'],len(items)),
                     'duration_sec':series_summary([float(r['duration_sec']) for r in items]),
                     'localization_stop_sec':series_summary([float(r['localization_stop_sec']) for r in items]),
                     'interventions':sum(int(r['interventions']) for r in items),
                     'map_hashes':sorted({r['map_hash'] for r in items}),
                     'parameter_hashes':sorted({r['parameter_hash'] for r in items}),
                     'reference_sources':sorted({r['reference_source'] for r in items})}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs'); parser.add_argument('--rows'); parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if not args.runs and not args.rows:
        parser.error('--runs or --rows required; no inferred mission success')
    report={'schema_version':1}
    if args.runs: report['runs']=run_metrics(args.runs)
    if args.rows: report['rows']=row_metrics(args.rows)
    output=Path(args.output)
    if output.exists(): parser.error('output already exists')
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream: json.dump(report,stream,indent=2,allow_nan=False)
    print(str(output.resolve()))
