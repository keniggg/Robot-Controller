from pathlib import Path
import importlib.util
import itertools
import numpy as np
import pytest

p=Path(__file__).with_name('benchmark_regional_composite_20261007.py')
spec=importlib.util.spec_from_file_location('composite_benchmark',p)
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)


@pytest.mark.parametrize('shape',[(48,64),(49,67),(480,640)])
def test_disjoint_original_ownership_counts_and_all_active_combinations(shape):
    rng=np.random.RandomState(17);current=rng.randint(0,65536,shape,dtype=np.uint16)
    arrays=[rng.randint(0,65536,shape,dtype=np.uint16) for _ in range(5)]
    rows=b.regions(shape);original=current.copy()
    owner=np.zeros(shape,np.uint8)
    for row in rows:
        x0,y0,x1,y1=row['bounds'];owner[y0:y1,x0:x1]+=row['owned']
    assert np.all(owner==1) and sum(row['owned_pixels']for row in rows)==current.size
    for active in itertools.product((False,True),repeat=5):
        expected,report=b.composite(current,arrays,rows,active,'original')
        for variant in ('cached_counts','cached_counts_copyto'):
            actual,newreport=b.composite(current,arrays,rows,active,variant)
            assert np.array_equal(actual,expected) and newreport==report
            assert not np.shares_memory(actual,current)
    assert np.array_equal(current,original)


def test_strided_uint16_filtered_views_exact():
    rng=np.random.RandomState(29);shape=(49,67)
    current=rng.randint(0,10000,shape,dtype=np.uint16)
    arrays=[rng.randint(0,10000,(shape[0],shape[1]*2),dtype=np.uint16)[:,::2]for _ in range(5)]
    assert not arrays[0].flags.c_contiguous
    rows=b.regions(shape)
    old,report=b.composite(current,arrays,rows,(True,)*5,'original')
    new,newreport=b.composite(current,arrays,rows,(True,)*5,'cached_counts_copyto')
    assert np.array_equal(new,old) and report==newreport


def test_unsupported_variant_rejected():
    with pytest.raises(ValueError):b.composite(None,None,None,None,'unknown')
