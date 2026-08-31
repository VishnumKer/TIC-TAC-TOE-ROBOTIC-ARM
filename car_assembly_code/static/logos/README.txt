Drop logo image files here, then update home_dashboard.html:

  mouser.png  → main logo slot (top, full width)
  sub1.png    → sub-logo top-left
  sub2.png    → sub-logo top-right
  sub3.png    → sub-logo bottom-left
  sub4.png    → sub-logo bottom-right

In home_dashboard.html, replace each <span class="logo-placeholder"> block
with an <img> tag, e.g.:
  <img src="/static/logos/mouser.png" alt="Mouser">
