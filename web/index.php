<?php 
$link = mysqli_connect("localhost", "arjuna", "arjuna", "db_robot");

if (!$link) {
	echo "Error: Unable to connect to MySQL.";
	exit;
}
?>
<!DOCTYPE html>
<html>
	<head>
		<link rel="stylesheet" href="style3.css">
		<title>Dinamika_99 Robot Controller</title>
		<meta http-equiv="refresh" content="10"/>
	</head>
	<body>
		<div class="body-wrapper">
			<div class="grid-1" id="title">Control Panel</div>
			<div class="grid-2">
			<table align="center">
					<thead>
						<tr>
							<th>ID</th>
							<th>Nama</th>
							<th>Status</th>
						</tr>
					</thead>
						<tbody>
						<?php
						$qr2 = mysqli_query($link, "SELECT * FROM tbl_robot");
						while($row = mysqli_fetch_array($qr2, MYSQLI_ASSOC)) {
						?>
						<tr>
							<td><?php echo $row['id']; ?></td>
							<td><?php echo $row['nama']; ?></td>
							<td><?php echo $row['status']; ?></td>
						</tr>
						<?php } ?>
					</tbody>
				</table>
			</div>
			<div class="grid-3" id="title">Dinamika_99</div>
			<div class="grid-4">
			<?php
				$qr1 = mysqli_query($link, "SELECT * FROM tbl_game");
				while($row = mysqli_fetch_array($qr1, MYSQLI_ASSOC)) {
				?>
				<h2>Posisi Dummy</h2>
				<p><?php echo $row['dummy1']."-".$row['dummy2']."-".$row['kiper']; ?></p>
				<form action="setdummy.php" method="post">
					<label for="dummy1">Dummy 1</label><br>
					<input class="dummy" type="text" id="dummy1" name="dummy1" value="<?php echo $row['dummy1']; ?>"><br><br>
					<label for="dummy2">Dummy 2</label><br>
					<input class="dummy" type="text" id="dummy2" name="dummy2" value="<?php echo $row['dummy2']; ?>"><br><br>
					<label for="kiper">Kiper</label><br>
					<input class="dummy" type="text" id="kiper" name="kiper" value="<?php echo $row['kiper']; ?>"><br><br>
					<input class="btn" type="submit" value="Submit">
				</form>
			</div>
			<div class="grid-5">
				<div>
					<h2>Mode</h2>
					<p><?php echo $row['mode']; ?></p>
					<form action="setmode1.php">
						<input class="btn" type="submit" value="Kickoff Kanan">
					</form>
					<form action="setmode2.php">
						<input class="btn" type="submit" value="Kickoff Kiri">
					</form>
					<form action="setmode3.php">
						<input class="btn" type="submit" value="Corner">
					</form>
				</div>
			</div>
			<div class="grid-6">
				<div>
					<h2>Game Status</h2>
					<p><?php echo $row['status']; ?></p>
					<form action="setgame1.php">
						<input class="btn" type="submit" value="Start">
					</form>
					<form action="setgame2.php">
						<input class="btn" type="submit" value="Retry">
					</form>
					<form action="setgame3.php">
						<input class="btn" type="submit" value="Stop">
					</form>
				</div>
			</div>
			<?php } ?>
			</div>
		</div>
	</body>
	<footer>
		Created By: @arjunapanji21 (8030170001). Copyright &copy;2021
	</footer>
</html>