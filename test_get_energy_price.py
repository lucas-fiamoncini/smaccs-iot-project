from entsoe import EntsoePandasClient
import pandas as pd

API_KEY = 'YOUR_ENTSOE_API_KEY'

client = EntsoePandasClient(api_key=API_KEY)

start = pd.Timestamp('20260514', tz='Europe/Helsinki')
end = pd.Timestamp('20260515', tz='Europe/Helsinki')
country_code = 'FI'  # Finland

content = client.query_day_ahead_prices(country_code, start, end)

df = pd.DataFrame(content).reset_index(drop=False).rename(columns={'index': 'timestamp', 0: 'price'})

print(df.head(10))