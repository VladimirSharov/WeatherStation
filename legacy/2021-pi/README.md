# 2021 Raspberry Pi code (original)

The receiver and web page as they ran on the Pi in December 2021, kept for history. They were replaced by [`receiver/`](../../receiver/). Database credentials were placeholders already, and nothing here has been edited.

- **`readdata.py`** read lines from the central Nucleo board on `/dev/ttyACM0`. It appended each reading to `/var/www/html/data.csv` and inserted it into a MySQL `Sensor` table.
- **`index.php`** showed the last 10 rows as an HTML table that reloaded every 3 seconds, with links to download the CSV and the project specification. Each row held temperature and humidity for two locations.

## Why it was replaced

- **Values were read by character position.** The script took temperature 1 from `data[4:6]`, humidity 1 from `data[11:13]`, and so on, so it expected a line shaped roughly like `T1: 23 H1: 45% T2: 22 H2: 50%`. A line that arrived shifted, cut short or garbled produced junk values or crashed the script with `IndexError`. Nothing checked whether a line was intact, and a single digit could be misread.
- **It only worked on Python 2.** `ser.readline()` returns `bytes`. Under Python 3, `data[0] == 'T'` compares an integer with a string and is always `False`, so no line would ever be processed. Python 2 was still the default `python` on Raspberry Pi OS at the time, even though it reached end of life in January 2020.
- **Values were whole numbers only**, two digits at most, with no negatives.
- **A failed database connection crashed the script.** The `finally` block then referenced a `cursor` that had never been created, raising `NameError`.

The new receiver uses a framed protocol with a station ID, a sequence number and a checksum (see [`receiver/protocol.py`](../../receiver/protocol.py)). It rejects and logs bad lines instead of storing them, and stores fixed-point values with two decimal places.
