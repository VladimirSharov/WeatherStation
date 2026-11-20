<!DOCTYPE html>
<html>
<head>

<style>
table {
  font-family: arial, sans-serif;
  border-collapse: collapse;
  width: 50%;
}

td, th {
  border: 1px solid #dddddd;
  text-align: left;
  padding: 8px;
}

tr:nth-child(even) {
  background-color: #dddddd;
}
body {
background-image: url('wall2.jpeg');
}
</style>
<meta http-equiv="refresh" content="3" >
</head>
<body>
<h1>Weather Station</h1>
<div>
<?php
$servername = "servername";
$username = "username";
$password = "password";
$dbname = "databasename";

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);
// Check connection
if ($conn->connect_error) {
  die("Connection failed: " . $conn->connect_error);
}

//$sql = "SELECT temp, hum, reading_time FROM Sensor ORDER BY id DESC LIMIT 10";
$sql = "SELECT * FROM (SELECT * FROM Sensor ORDER BY id DESC LIMIT 10) sub ORDER by id ASC";
$result = $conn->query($sql);
echo "<table>";
echo "<tr>";
echo "<th>Time</th>";
echo "<th>Temperature °C (Loc 1)</th>";
echo "<th>Humidity % (Loc 1)</th>";
echo "<th>Temperature °C (Loc 2)</th>";
echo "<th>Humidity % (Loc 2)</th>";
echo "</tr>";



if ($result->num_rows > 0) {
  // output data of each row
  while($row = $result->fetch_assoc()) {
    //echo " Temperature: " . $row["temp"]. " Humidity: " . $row["hum"]. " Time: " . $row["reading_time"]."<br>";
	echo "<tr>" . "<td>" . $row["reading_time"] . "</td>". "<td>" . $row["temp"] . "</td>" . "<td>" . $row["hum"] . "</td>" . "<td>" . $row["temp2"] . "</td>" . "<td>" .  $row["hum2"] . "</td>"  . "</tr>";
 }
} else {
  echo "0 results";
}
echo "</table>";

$conn->close();
?>
</div>

<div>
<p>
<a href="data.csv">Download data</a>
</p>
<p>
<a target = '_blank' href="srs_spec.pdf">Download specification</a>
</p>
</div>

</body>
</html>
