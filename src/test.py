import pickle
import pandas as pd

df = pickle.load( open( "save.p", "rb" ) )
print(df.head(60))
print(df.info())