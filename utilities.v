module utilities();

function overflow;
	input cur, prev;
	begin
		overflow = (cur[$high(cur)] == prev[$high(prev)]) && (
